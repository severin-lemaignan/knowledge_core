#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from distutils.core import setup

version = {}
with open("src/minimalkb/__init__.py") as fp:
    exec(fp.read(), version)


def readme():
    with open("README-pypi.rst") as f:
        return f.read()


setup(
    name="minimalKB",
    version=version["__version__"],
    license="BSD",
    description="A RDFlib-backed minimalistic knowledge based for robotic application",
    long_description=readme(),
    classifiers=[
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python :: 3",
    ],
    author="Séverin Lemaignan",
    author_email="severin.lemaignan@pal-robotics.com",
    url="https://github.com/severin-lemaignan/minimalkb",
    requires=["rdflib"],
    package_dir={"": "src"},
    packages=["minimalkb"],
    scripts=["bin/minimalkb"],
    data_files=[
        (
            "share/ontologies",
            ["share/ontologies/" + f for f in os.listdir("share/ontologies")],
        ),
        ("share/doc/minimalkb", ["LICENSE", "README.md"]),
    ],
)
