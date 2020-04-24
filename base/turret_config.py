
#UNDER DEVELOPMENT

import os 
import os.path as osp 
import numpy as np 
from easydict import EasyDict as edict 

def turret_config():
    cfg = {}
    cfg['slave_address'] = 0x08

    # Arduino Data Size
    cfg['arduino_data_size'] = 12

    cfg['fire_cmd'] = 0 # OR 1

    cfg['rot_on'] = 0  # OR 1
    cfg['rot_dir'] = 0 # OR 1
    cfg['rot_steps'] = 0/10 # OR -> No of steps
    cfg['rot_delay'] = 0/100 #  OR (#us)/100

    cfg['pit_on'] = 0 # OR 1
    cfg['pit_dir'] = 0 # OR 1
    cfg['pit_steps'] = 0/10 # OR #steps/10
    cfg['pit_delay'] = 0/100 # OR (us)/100
    return cfg
