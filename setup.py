#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from distutils.core import setup

version = {}
with open("src/knowledge_core/__init__.py") as fp:
    exec(fp.read(), version)


def readme():
    with open("README-pypi.rst") as f:
        return f.read()


setup(
    name="KnowledgeCore",
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
    url="https://github.com/severin-lemaignan/knowledge_core",
    requires=["rdflib"],
    package_dir={"": "src"},
    packages=["knowledge_core"],
    # scripts=["bin/knowledge_core"], # commented out, as bin installed via catkin in ROS world
    data_files=[
        (
            "share/ontologies",
            ["share/ontologies/" + f for f in os.listdir("share/ontologies")],
        ),
        ("share/doc/knowledge_core", ["LICENSE", "README.md"]),
    ],
)
