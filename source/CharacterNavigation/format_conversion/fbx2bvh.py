"""
This code comes from https://github.com/rubenvillegas/cvpr2018nkn/blob/master/datasets/fbx2bvh.py
"""
import bpy
import numpy as np

import os
import sys
sys.path.append(os.getcwd())
sys.path.append('../')
from os import listdir
import math

data_path = './'

files = sorted([f for f in listdir(data_path) if f.endswith(".fbx")])

for f in files:
    sourcepath = data_path + f
    dumppath = data_path + f.split(".fbx")[0] + ".bvh"

    bpy.ops.import_scene.fbx(filepath=sourcepath, axis_forward='Y', axis_up='Z')


    action = bpy.data.actions[-1]
    frame_start = math.floor(action.frame_range[0])
    frame_end = math.floor(action.frame_range[1])

    bpy.ops.export_anim.bvh(filepath=dumppath,
                            frame_start=frame_start,
                            frame_end=frame_end, root_transform_only=True, rotate_mode='XYZ')
    bpy.data.actions.remove(bpy.data.actions[-1])

    print(data_path + f + " processed.")

bpy.ops.wm.quit_blender()
