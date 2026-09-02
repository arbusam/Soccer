"""
Build script for the lidar pybind11 module.

Usage (from project root):
    python lib/setup.py build_ext --inplace
"""

import json
import sysconfig
from pathlib import Path

import pybind11
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

lib_dir = Path(__file__).resolve().parent
project_root = lib_dir.parent

# Get pybind11 include path
pybind11_include = pybind11.get_include()

# SDK paths (relative to project root)
sdk_include = str(project_root / "rplidar_sdk/sdk/include")
sdk_src = str(project_root / "rplidar_sdk/sdk/src")
sdk_lib = str(project_root / "rplidar_sdk/output/Linux/Release")

# Define the extension module
lidar_module = Extension(
    'lib.lidar',
    sources=[
        str(lib_dir / 'lidar_module.cpp'),
        str(lib_dir / 'localisation.cpp'),
    ],
    include_dirs=[
        pybind11_include,
        sdk_include,
        sdk_src,
        str(lib_dir),
    ],
    library_dirs=[sdk_lib],
    libraries=['sl_lidar_sdk', 'pthread', 'rt'],
    extra_compile_args=['-std=c++11', '-O2', '-fPIC'],
    language='c++',
)


class BuildExtWithCompileCommands(build_ext):
    """Build the extension and emit compile_commands.json for IDE/clangd."""

    def build_extensions(self):
        super().build_extensions()
        python_include = sysconfig.get_path("include")
        commands = []
        for ext in self.extensions:
            compile_flags = list(ext.extra_compile_args or [])
            for include_dir in ext.include_dirs:
                compile_flags.append(f"-I{include_dir}")
            compile_flags.append(f"-I{python_include}")
            for source in ext.sources:
                source_path = Path(source)
                commands.append(
                    {
                        "directory": str(project_root),
                        "command": " ".join(
                            ["g++", *compile_flags, "-c", source_path.name]
                        ),
                        "file": str(source_path),
                    }
                )
        (project_root / "compile_commands.json").write_text(
            json.dumps(commands, indent=2) + "\n"
        )


setup(
    name='lib.lidar',
    version='1.0',
    description='RPLidar C1 Python module',
    ext_modules=[lidar_module],
    cmdclass={'build_ext': BuildExtWithCompileCommands},
)
