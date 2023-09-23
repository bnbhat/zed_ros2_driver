#-----------------------------------------------------------------------------------
# file: utils.py
# description: utils for ZED Driver
# author: Balachandra Bhat <bnbhat311@gmail.com>
# created: 2023-09-22
# last modified: 2023-09-23
#-----------------------------------------------------------------------------------

import time
import pyzed.sl as sl


ID_COLORS = [(232, 176,59)
            ,(175, 208,25)
            ,(102, 205,105)
            ,(185, 0,255)
            ,(99, 107,252)]


def render_object(object_data, is_tracking_on) -> bool:
    if is_tracking_on:
        return (object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OK)
    else:
        return ((object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OK) or (object_data.tracking_state == sl.OBJECT_TRACKING_STATE.OFF))
        

def generate_color_id_u(idx) -> list:
    arr = []
    if(idx < 0):
        arr = [236,184,36,255]
    else:
        color_idx = idx % 5
        arr = [ID_COLORS[color_idx][0], ID_COLORS[color_idx][1], ID_COLORS[color_idx][2], 255]
    return arr

def calculate_fps(func):
    def wrapper(*args, **kwargs):
        print("Calculating FPS...")
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        fps = 1.0 / (end - start)
        print("FPS: ", fps)
        return result
    return wrapper