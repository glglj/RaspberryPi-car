"""编译 Cython 扫描匹配器。
从项目根目录运行:  python slam/setup_scan.py build_ext --inplace
"""
import os
from setuptools import setup, Extension
from Cython.Build import cythonize

_HERE = os.path.dirname(os.path.abspath(__file__))

setup(
    ext_modules=cythonize(
        Extension(
            "slam.scan_matcher_fast",
            [os.path.join(_HERE, "scan_matcher_fast.pyx")],
        ),
        compiler_directives={"language_level": "3"},
    ),
)
