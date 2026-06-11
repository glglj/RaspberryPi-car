"""编译 IMU Cython parser。
从项目根目录运行:  cd ~/Desktop/raspberry_car && python imu/setup_imu.py build_ext --inplace
"""
import os
from setuptools import setup, Extension
from Cython.Build import cythonize

_HERE = os.path.dirname(os.path.abspath(__file__))

setup(
    ext_modules=cythonize(
        Extension("imu.imu_parser", [os.path.join(_HERE, "imu_parser.pyx")]),
        compiler_directives={"language_level": "3"},
    ),
)