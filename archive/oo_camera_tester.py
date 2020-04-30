# -*- coding: utf-8 -*-
"""
Created on Wed Apr 15 11:24:00 2020

@author: chadd
"""

import smbus
import sys
from time import time
from time import sleep
import argparse
from argparse import ArgumentParser
import json
import numpy as np
import cv2
# from __future__ import print_function
from difflib import SequenceMatcher
import depthai
import simple_pid
import consts.resource_paths
from depthai_helpers import utils
from targeting_color_oo import Targeting



lux = Targeting()

lux.activate(imshow_debug=True, communication_on=False)





