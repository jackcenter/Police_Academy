
#UNDER DEVELOPMENT

import os 
import os.path as osp 
import numpy as np 
from easydict import EasyDict as edict 

def base_model_config():
    
    cfg = edict()

    #Radius of the wheel in inches
    cfg.radius = 2

    # Distance between the wheels 
    cfg.axel = 10

    #Encoder Specification -> Counts/Clicks per revolution
    cfg.cpr = 8400

    #Number of Encoder count per single turn 
    cfg.res = 64

    #Ultrasonic Configuration 
    cfg.TRIG_1 = 18
    cfg.ECHO_1 = 19

    cfg.TRIG_2 = 20
    cfg.ECHO_2 = 21

    cfg.TRIG_3 = 22
    cfg.ECHO_3 = 23

    cfg.TRIG_PINS = [TRIG_1, TRIG_2, TRIG_3]
    cfg.ECHO_PINS = [ECHO_1, ECHO_2, ECHO_3]

    cfg.units = 'in'

    #Chassis configuration 
    cfg.slave_address = 0x07 


    return cfg
