"""编译 IMU Cython parser。在树莓派上运行: python setup_imu.py build_ext --inplace"""
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize(
        "imu_parser.pyx",
        compiler_directives={"language_level": "3"},
    ),
)