# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import re

from setuptools import setup

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("ktune/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


requirements_dev = [
    "pytest>=7.0.0",
    "black>=22.0.0",
    "mypy>=1.0.0",
    "ruff>=0.1.0",
]

requirements_examples = [
    "matplotlib>=3.0.0",
    "numpy>=1.20.0",
]


with open("ktune/__init__.py", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^__version__ = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in kos_sim/__init__.py"
version: str = version_re.group(1)


setup(
    name="ktune",
    version=version,
    description="Servo Control Tuning Tool for Real and Simulated Actuators. Utilizes Kscale KOS and KOS-SIM",
    author="Scott",
    author_email="scott@whoopnet.ai",
    url="https://github.com/nfreq/ktune",
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    install_requires=requirements,
    tests_require=requirements_dev,
    extras_require={
        "dev": requirements_dev,
        "examples": requirements_examples,
    },
    packages=["ktune"],
    include_package_data=True,
    package_data={
        "ktune": ["requirements.txt"],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.11",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    entry_points={
        "console_scripts": [
            "ktune=ktune.ktune:cli",
        ],
    },
)
