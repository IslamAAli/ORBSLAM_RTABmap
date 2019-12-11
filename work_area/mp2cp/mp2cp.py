#%%
'''
To generate point clouds and the keyframe associated with them.
Command: python mp2cp.py --mappath='./DS_FinalMapDetailed.txt' --keyframepath='./DS_keyFrameTransformations_RM.txt' --savepath='./point_cloud/' --first_file='rgb.txt' --second_file='depth.txt'  
'''

import numpy as np
import pcl
import shutil 
import sys
import os
import argparse

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

class pointcloud:
    def __init__(self,keyframe,points):
        self.timestamp = keyframe.timestamp
        self.keyframe = keyframe
        self.points = points
        self.pointscloud = list()
    
    def global2local(self,point,rotation,translation):
        rotation_matrix = np.array(rotation).reshape([3,3])
        translation = np.array(translation)
        point = np.array([point.x,point.y,point.z])
        local_point = rotation_matrix.dot(point)+translation
        return local_point
    
    def cloud(self):
        for i in self.points:
            for j in i.frames:
                if j==self.keyframe.keyframeid:
                    local_point = self.global2local(i,self.keyframe.rotation,self.keyframe.translation)
                    self.pointscloud.append(local_point)
        self.pointscloud=np.reshape(self.pointscloud,[-1,3])

def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    
    matches.sort()
    return matches


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arguments for running.')
    parser.add_argument('--first_file',type=str,default='', help='first text file (format: timestamp data)')
    parser.add_argument('--second_file',type=str,default='', help='second text file (format: timestamp data)')
    parser.add_argument('--mappath', type=str,default='',help='Specify detailed map path')
    parser.add_argument('--keyframepath', type=str,default='',help='Specify detailed keyframe path')
    parser.add_argument('--savepath', type=str,default='',help='Specify save file path')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)

    args = parser.parse_args()
    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)
    mappath = args.mappath
    keyframepath = args.keyframepath
    savepath = args.savepath
    if not os.path.exists("rgb_sync"): 
        os.makedirs("rgb_sync")
    if not os.path.exists("depth_sync"): 
        os.makedirs("depth_sync")
    if not os.path.exists("point_cloud"): 
        os.makedirs("point_cloud")
    # %%
    # f = open(filepath)
    # self.points = dict.fromkeys(["x","y","z","frames"])
    # for i in f:
    #     temp = i.split
    #     self.points["x"] = temp[0]
    #     self.points["y"] = temp[1]
    #     self.points["z"] = temp[2]
    #     self.points["frames"] = temp[3:]
    with open(mappath) as f:
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
    with open(keyframepath) as f:
        k = list()
        d = {}
        for i in f:
            temp = i
            temp = temp.split()
            d["timestamp"] = float(temp[0])
            d["translation"] = list(map(float,temp[1:4]))
            d["rotation"] = list(map(float,temp[4:13]))
            d["keyframeid"] = float(temp[13])
            d["intrinsinc"] = list(map(float,temp[15:-1]))
            k.append(keyframe(d))

    times = list()

    # %%
    for kk in k:
        pp = pointcloud(kk,points)

    # %%
        pp.cloud()
        savepath1 = savepath + str(kk.keyframeid)
        savepath2 = savepath1 + ".pcd"

    # %%
        p = pcl.PointCloud(len(pp.pointscloud))
        p.from_array(np.float32(pp.pointscloud))
        if p.width != 0:
            np.savetxt(savepath1,pp.pointscloud)
            pcl.save(p,savepath2,binary=False)
        times.append(kk.timestamp)
        print(len(times))
    
    for j in times:
        first_keys = first_list.keys()
        # print(first_keys[0])
        second_keys = second_list.keys()
        # a = np.array(first_keys)
        off = [(abs(jj-j),jj) for jj in first_keys]
        temp = 100000000
        for a,b in off:
            if a<temp:
                key1 = b
                temp = a
        
        shutil.copyfile(" ".join(first_list[key1]), "rgb_sync/" + " ".join(first_list[key1]).split("/")[1])
        # b = np.array(second_keys)
        off = [(abs(jj-j),jj) for jj in second_keys]
        temp = 100000000
        for a,b in off:
            if a<temp:
                key2 = b
                temp = a

        shutil.copyfile(" ".join(second_list[key2]), "depth_sync/" + " ".join(second_list[key2]).split("/")[1])

