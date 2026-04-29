
#用于部署cython
from setuptools import setup
from Cython.Build import cythonize

## cd /home/pi/Desktop/raspberry_car/lidar
 ## python3 setup.py build_ext --inplace

setup(
    name="lidar_parser",
    ext_modules=cythonize(
        "lidar_parser.pyx",
        compiler_directives={"language_level": "3"},
    ),
)
