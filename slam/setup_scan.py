"""编译 Cython SLAM 加速模块。
从项目根目录运行:  python slam/setup_scan.py build_ext --inplace
"""
import os
from setuptools import setup, Extension
from Cython.Build import cythonize

_HERE = os.path.dirname(os.path.abspath(__file__))

setup(
    ext_modules=cythonize(
        [
            Extension(
                "slam.scan_matcher_fast",
                [os.path.join(_HERE, "scan_matcher_fast.pyx")],
            ),
            Extension(
                "slam.local_mapper_fast",
                [os.path.join(_HERE, "local_mapper_fast.pyx")],
            ),
        ],
        compiler_directives={"language_level": "3"},
    ),
)
