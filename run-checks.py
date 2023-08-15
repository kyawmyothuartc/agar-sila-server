#!/usr/bin/env python
import os
import sys

for command in [
    f"{sys.executable} -m isort --check-only .",
    f"{sys.executable} -m black --check .",
    f"{sys.executable} -m pflake8 .",
    # f"{sys.executable} -m pytest --cov=src --cov-config=pyproject.toml tests/",
]:
    print(f"Running {command!r}")
    exit_code = os.system(command)
    if exit_code != 0:
        exit(exit_code)
