# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""A setuptools based setup module.

See:
https://packaging.python.org/en/latest/distributing.html
https://github.com/pypa/sampleproject
"""

from setuptools import setup, find_packages

# To use a consistent encoding
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, "README.rst"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="fergcorp-circuitpython-pca9745b-spi",
    use_scm_version=True,
    setup_requires=["setuptools_scm"],
    description="SPI driven CircuitPython driver for PCA9745B constant current LED driver.",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    # The project's main homepage.
    url="https://github.com/fergcorp/CircuitPython_PCA9745B_SPI",
    # Author details
    author="Fergcorp, LLC",
    author_email="andrew@fergcorp.com",
    install_requires=[
        "adafruit-circuitpython-busdevice",
    ],
    # Choose your license
    license="MIT",
    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Hardware",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.8",
    ],
    # What does your project relate to?
    keywords="adafruit circuitpython pca9745b_spi pca9745b led-driver",
    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    # TODO: IF LIBRARY FILES ARE A PACKAGE FOLDER,
    #       CHANGE `py_modules=['...']` TO `packages=['...']`
    py_modules=["pca9745b_spi"],
)
