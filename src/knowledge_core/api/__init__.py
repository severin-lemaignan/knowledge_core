#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import random
import shlex


class KbError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class KB:
    def __init__(self):
        pass
