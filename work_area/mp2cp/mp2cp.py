#%%
import numpy as np
import pcl

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

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Arguments for running.')
    parser.add_argument('--mappath', type=str,default='',help='Specify detailed map path')
    parser.add_argument('--keyframepath', type=str,default='',help='Specify detailed keyframe path')
    parser.add_argument('--savepath', type=str,default='',help='Specify save file path')

    args = parser.parse_args()
    mappath = args.mappath
    keyframepath = args.keyframepath
    savepath = args.savepath
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
            pcl.save(p,savepath2)
            np.savetxt(savepath1,pp.pointscloud)

