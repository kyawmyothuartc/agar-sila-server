#!/usr/bin/env python
import os
import sys

for command in [
    f"{sys.executable} -m isort .",
    f"{sys.executable} -m black .",
]:
    print(f"Running {command!r}")
    exit_code = os.system(command)
    if exit_code != 0:
        exit(exit_code)
