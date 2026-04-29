# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True

from libc.stdint cimport uint8_t

cdef class LidarParser:
    cdef uint8_t buffer[8192]
    cdef int buf_len

    def __cinit__(self):
        self.buf_len = 0

    cpdef list feed(self, bytes data):
        cdef int n = len(data)
        cdef int i = 0
        cdef list result = []
        cdef int k, remaining, packet_len, lsn
        cdef int cs_calc, cs_received
        cdef uint8_t si_l, si_2nd, si_h
        cdef int distance, intensity, high_ref
        cdef list si_list
        cdef int fsa_raw, lsa_raw
        cdef double angle_fsa, angle_lsa, angle_step, angle

        # 拷贝数据到 buffer
        if self.buf_len + n > 8192:
            self.buf_len = 0
        for k in range(n):
            self.buffer[self.buf_len + k] = data[k]
        self.buf_len += n

        while i + 10 <= self.buf_len:
            if self.buffer[i] != 0xAA or self.buffer[i+1] != 0x55:
                i += 1
                continue

            lsn = self.buffer[i+3]
            packet_len = 10 + lsn*3
            if i + packet_len > self.buf_len:
                break

            # --- CS 校验 ---
            cs_calc = 0
            # CSL: 校验位之前所有字节（第三位 M&T 到 LSA）两两 XOR
            for k in range(0, 8, 2):
                val = self.buffer[i + k] | (self.buffer[i + k + 1] << 8)
                cs_calc ^= val

            # CSH: 每三个字节，前两个字节组合成16位小端整数异或
            for k in range(10, 10 + lsn * 3, 3):
                val1 = self.buffer[i + k] | (self.buffer[i + k + 1] << 8)  # Si_L, Si_2nd
                val2 = self.buffer[i + k + 2] | (0x00 << 8)  # Si_H, 高8位补0
                cs_calc ^= val1
                cs_calc ^= val2

            # 取低16位（小端存储）
            cs_calc &= 0xFFFF
            cs_received = self.buffer[i + 8] | (self.buffer[i + 9] << 8)

            # if cs_calc != cs_received:
            #     i += 1
            #     continue

            # --- 角度 ---
            fsa_raw = self.buffer[i+4] | (self.buffer[i+5]<<8)
            lsa_raw = self.buffer[i+6] | (self.buffer[i+7]<<8)
            angle_fsa = (fsa_raw >> 1) / 64.0
            angle_lsa = (lsa_raw >> 1) / 64.0
            if lsn > 1:
                angle_step = (angle_lsa - angle_fsa) / (lsn - 1)
            else:
                angle_step = 0

            si_list = []
            for k in range(lsn):
                base = i + 10 + k*3
                si_l = self.buffer[base]
                si_2nd = self.buffer[base+1]
                si_h = self.buffer[base+2]

                distance = si_h*64 + (si_2nd >> 2)
                intensity = (si_2nd & 0x03)*64 + (si_l >> 2)
                high_ref = si_l & 0x01
                angle = angle_fsa + k*angle_step

                si_list.append({
                    "distance": distance,
                    "intensity": intensity,
                    "high_ref": high_ref,
                    "angle": angle
                })

            result.append({
                "LSN": lsn,
                "FSA_raw": fsa_raw,
                "LSA_raw": lsa_raw,
                "angle_fsa": angle_fsa,
                "angle_lsa": angle_lsa,
                "start_idx": i,
                "packet_len": packet_len,
                "Si": si_list,

            })

            i += packet_len

        # 剩余数据移动
        if i > 0:
            remaining = self.buf_len - i
            for k in range(remaining):
                self.buffer[k] = self.buffer[i+k]
            self.buf_len = remaining

        return result
