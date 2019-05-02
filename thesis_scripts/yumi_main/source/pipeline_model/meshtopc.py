#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2018 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the BSD license.

"""

"""

#

import numpy as np
import open3d
from open3d import PointCloud, Vector3dVector, write_point_cloud,read_triangle_mesh
from open3d import *
import time
from pyntcloud import PyntCloud


#from mesh to point cloud data
# meshtopcd="rightFace_cm.ply"
# output_="pc_render_cm"

# meshtopcd="rightFace_m.ply"
# output_="rightFace_m_down"

meshtopcd="front_face_m.ply"
output_="front_face_m_down"


# meshtopcd="rightFace45position_m.ply"
# output_="rightFace45position_render"

cloud = PyntCloud.from_file(meshtopcd)
new_cloud = cloud.get_sample('mesh_random',n=10200, rgb=False, normals=False)
print(new_cloud.shape)


#save the point cloud data file wiht the help of open3d
new_cloud=np.nan_to_num(new_cloud)
pcd = PointCloud()
print('In progress!!!')
start_timer=time.time()
pcd.points = Vector3dVector(new_cloud)
write_point_cloud(output_+".pcd", pcd)
write_point_cloud(output_+".ply", pcd)
print('elapsed time:',time.time()-start_timer)


#visualization
source =read_point_cloud(output_+".pcd")
print(type(source))
tmp_source=np.asarray(source.points)
print('shape:',tmp_source.shape)
# Flip it, otherwise the pointcloud will be upside down
#source.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
draw_geometries([source])
