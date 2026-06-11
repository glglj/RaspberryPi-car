# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True

from libc.stdint cimport uint8_t, int16_t


cdef class ImuParser:
    """IMU 帧解析器（Cython 加速）。自动将 0x51→0x59 递增序列打包为一个周期。"""
    cdef uint8_t buffer[4096]
    cdef int buf_len
    cdef list _bundle          # 当前周期的帧列表
    cdef int _last_type        # 上一条帧的类型，用于检测周期边界

    def __cinit__(self):
        self.buf_len = 0
        self._bundle = []
        self._last_type = 0

    cpdef list feed(self, bytes data):
        """喂入原始字节，返回完整周期列表。
        每个周期是一个 list[dict]，按 0x51→0x52→0x53→0x54→0x59 顺序排列。
        跨周期的残余帧留在内部 _bundle 中，等待下次凑齐。
        """
        cdef int n = len(data)
        cdef int i = 0
        cdef int k, remaining
        cdef list result = []
        cdef uint8_t dtype, crc, calc
        cdef int16_t raw0, raw1, raw2, raw3
        cdef float v0, v1, v2, v3

        # 追加到内部 C buffer
        if self.buf_len + n > 4096:
            self.buf_len = 0
        for k in range(n):
            self.buffer[self.buf_len + k] = data[k]
        self.buf_len += n

        while i + 11 <= self.buf_len:
            # 帧头 0x55
            if self.buffer[i] != 0x55:
                i += 1
                continue

            dtype = self.buffer[i + 1]
            if not (0x50 <= dtype <= 0x5F):
                i += 1
                continue

            # CRC: sum of first 10 bytes
            calc = 0
            for k in range(i, i + 10):
                calc += self.buffer[k]
            calc &= 0xFF
            crc = self.buffer[i + 10]
            if calc != crc:
                i += 1
                continue

            # ---- 周期边界检测 ----
            # 帧类型从 0x51 到 0x59 递增，如果新的 type <= 上次的 type，
            # 说明 IMU 已经开始了新的一轮
            if dtype <= self._last_type and self._bundle:
                result.append(self._bundle)
                self._bundle = []

            # 根据类型解析 data[2:10]
            raw0 = <int16_t>(self.buffer[i + 2] | (self.buffer[i + 3] << 8))
            raw1 = <int16_t>(self.buffer[i + 4] | (self.buffer[i + 5] << 8))
            raw2 = <int16_t>(self.buffer[i + 6] | (self.buffer[i + 7] << 8))
            raw3 = <int16_t>(self.buffer[i + 8] | (self.buffer[i + 9] << 8))

            if dtype == 0x51:          # ACC
                v0 = raw0 * (16.0 / 32768.0)
                v1 = raw1 * (16.0 / 32768.0)
                v2 = raw2 * (16.0 / 32768.0)
                v3 = raw3 / 100.0
                self._bundle.append({"type": dtype, "ax": v0, "ay": v1, "az": v2, "temp": v3})
            elif dtype == 0x52:        # GYRO
                v0 = raw0 * (2000.0 / 32768.0)
                v1 = raw1 * (2000.0 / 32768.0)
                v2 = raw2 * (2000.0 / 32768.0)
                self._bundle.append({"type": dtype, "gx": v0, "gy": v1, "gz": v2})
            elif dtype == 0x53:        # ANGLE
                v0 = raw0 * (180.0 / 32768.0)
                v1 = raw1 * (180.0 / 32768.0)
                v2 = raw2 * (180.0 / 32768.0)
                self._bundle.append({"type": dtype, "roll": v0, "pitch": v1, "yaw": v2})
            elif dtype == 0x54:        # MAG
                self._bundle.append({"type": dtype, "mx": raw0, "my": raw1, "mz": raw2})
            elif dtype == 0x59:        # QUAT
                v0 = raw0 / 32768.0
                v1 = raw1 / 32768.0
                v2 = raw2 / 32768.0
                v3 = raw3 / 32768.0
                self._bundle.append({"type": dtype, "q0": v0, "q1": v1, "q2": v2, "q3": v3})

            self._last_type = dtype
            i += 11

        # 残留数据移到 buffer 头部
        if i > 0:
            remaining = self.buf_len - i
            for k in range(remaining):
                self.buffer[k] = self.buffer[i + k]
            self.buf_len = remaining

        return result

    cpdef object flush(self):
        """强制返回当前未完成的残余帧（超时兜底用），返回 None 表示没有残余。"""
        if self._bundle:
            bundle = self._bundle
            self._bundle = []
            self._last_type = 0
            return bundle
        return None