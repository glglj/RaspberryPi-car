# 用于部署 cython 编译
# cd /home/raspicar/Desktop/raspberry_car
# python3 lidar/setup.py build_ext --inplace

import os
from setuptools import setup, Extension
from Cython.Build import cythonize

BASE = os.path.dirname(os.path.abspath(__file__))

setup(
    name="lidar_parser",
    ext_modules=cythonize(
        Extension(
            "lidar.lidar_parser",                    # 完整限定名 → .so 放在 lidar/ 下
            [os.path.join(BASE, "lidar_parser.pyx")],  # .pyx 绝对路径
        ),
        compiler_directives={"language_level": "3"},
    ),
)
