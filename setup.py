"""
Build script for the lidar pybind11 module.

Usage:
    python setup.py build_ext --inplace
"""

import os
import sys
from setuptools import setup, Extension
import pybind11

# Get pybind11 include path
pybind11_include = pybind11.get_include()

# SDK paths
sdk_include = "./rplidar_sdk/sdk/include"
sdk_src = "./rplidar_sdk/sdk/src"
sdk_lib = "./rplidar_sdk/output/Linux/Release"

# Define the extension module
lidar_module = Extension(
    'lidar',
    sources=['lidar_module.cpp'],
    include_dirs=[
        pybind11_include,
        sdk_include,
        sdk_src,
    ],
    library_dirs=[sdk_lib],
    libraries=['sl_lidar_sdk', 'pthread', 'rt'],
    extra_compile_args=['-std=c++11', '-O2', '-fPIC'],
    language='c++',
)

setup(
    name='lidar',
    version='1.0',
    description='RPLidar C1 Python module',
    ext_modules=[lidar_module],
)

