#%%
import pandas as pd
import numpy as np

# %%
class point:
    def __init__(self,x=0,y=0,z=0,frames=0):
        self.x = x
        self.y = y
        self.z = z
        self.frames = frames

class keyframe:
    def __init__(self,dict):
        self.timestamp = dict["timestamp"]
        self.translation = dict["translation"]
        self.rotation = dict["rotation"]
        self.keyframeid = dict["keyframeid"]
        self.intrinsinc = dict["intrinsinc"]


# %%
# f = open(filepath)
# self.points = dict.fromkeys(["x","y","z","frames"])
# for i in f:
#     temp = i.split
#     self.points["x"] = temp[0]
#     self.points["y"] = temp[1]
#     self.points["z"] = temp[2]
#     self.points["frames"] = temp[3:]
with open('DS_FinalMapDetailed.txt') as f:
    points = list()
    d = {}
    for i in f:
        temp = i
        temp = temp.split()
        x = float(temp[0])
        y = float(temp[1])
        z = float(temp[2])
        frames = list(map(float,temp[3:-1]))
        points.append(point(x,y,z,frames))
#%%
with open('DS_keyFrameTransformations_RM.txt') as f:
    k = list()
    d = {}
    for i in f:
        temp = i
        temp = temp.split()
        d["timestamp"] = float(temp[0])
        d["translation"] = list(map(float,temp[1:4]))
        d["rotation"] = list(map(float,temp[4:13]))
        d["keyframeid"] = list(temp[13])
        d["intrinsinc"] = list(map(float,temp[15:-1]))
        k.append(keyframe(d))

# %%
