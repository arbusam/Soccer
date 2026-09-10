"""Hardware-free checks for scan timing and native motion uncertainty."""

import subprocess
import tempfile
import unittest
from pathlib import Path


class LocalisationMotionTests(unittest.TestCase):
    def test_native_timing_and_motion(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / "localisation-motion")
            subprocess.run(
                ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                 "-pthread", str(root / "tests/localisation_motion_native.cpp"),
                 "-o", executable], check=True,
            )
            subprocess.run([executable], check=True, timeout=30)
