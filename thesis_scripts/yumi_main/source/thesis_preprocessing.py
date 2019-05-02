#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2018 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the BSD license.

"""
Everthing done with open3d, pyntcloud, CloudCompare, meshLab and python_pcl binding (wrapper)
--------------------------------------------------------------------------------------------
#Python 2.7 anaconda in a virtual environment working with ros, and extra "tensorflow"
#In order to change the format .obj(CAD model) to .ply(triangle mesh) do the following:
1_open Cloud compare and save a copy as .ply extension
#From .ply (triangle mesh) to .pcd(sample points) run the following code(meshtopc.py),
# meshtopc.py use the a pyntcloud that only run in python3 where i used the method to
# sample point from the mesh data, also use open3D in order to save as .pcd the sample points.
#finally, for processing the data i am working with python_pcl
-----------------------------------------------------------------------------------------
"""

import sys
sys.path.insert(0, '/home/casch/yumi_depends_ws/src/thesis_pkg/yumi_main/scripts/project')

#The order matters
from thesis_library import *
from thesis_class import *
from thesis_function import *

#folder where my point cloud data
pc_path='pcdata/'
rgbd_path='pcd_from_color_depth/'


def display_objects(cloud1,cloud2):
    pcloud1 = read_point_cloud(cloud1)
    pcloud2 = read_point_cloud(cloud2)
    print("Showing pcloud1 (red) and pcloud2 (gray): ")
    pcloud1.paint_uniform_color([1, 0, 0])
    pcloud2.paint_uniform_color([0.8, 0.8, 0.8])

    #draw_geometries([pcloud1, pcloud2])
    draw_geometries([pcloud2])


def main():
    counter1=0
    counter2=0
    counter3=0


    #path_cloud='end_cloud_rgbd/'
    path_cloud='end_cloud_pc/'
    downsample_name=path_cloud+'downsample_name'

    roi_name=path_cloud+'roi_name'
    table_name=path_cloud+'table_name'
    objects_name=path_cloud+'objects_name'


    import sys
    print "This is the name of the script: ", sys.argv[0]
    flag=True

    rate = rospy.Rate(10) # 10hz
    #flag1=sys.argv[1]
    while not rospy.is_shutdown():
        counter1+=1

        # Load the point cloud from memory##read_point_cloud(rgbd)

        #rgbd_ = [pcl.load(rgbd) for rgbd in glob.glob(pc_path+'*pcd')]

        #At present with the reals sense camera we do not have point cloud but i did my work ouround with RGB-D
        #Wednesday 20-March-2019
        rgbd_ = [pcl.load(rgbd) for rgbd in glob.glob(pc_path+'*pcd')]
        #rgbd_ = [pcl.load(rgbd) for rgbd in glob.glob(rgbd_path+'*pcd')]

        if flag:
            for i, cloud in enumerate(rgbd_):

                # mask out point cloud in order to get only information in our region of interest, as we don't care about the other parts
                #filter = do_passthrough_filter(point_cloud = cloud,name_axis = 'x', min_axis = -0.5, max_axis = 0.5)

                # #Threshold when working with the realsense
                # filter = do_passthrough_filter(point_cloud = cloud,name_axis = 'z', min_axis = -0.70, max_axis = 1)

                #--------------------------------------------------------
                # Threshold when working with the astra
                filter = do_passthrough_filter(point_cloud = cloud,name_axis = 'x', min_axis = 0.25, max_axis = 0.75)
                filter = do_passthrough_filter(point_cloud = filter,name_axis = 'y', min_axis = -0.10, max_axis = 0.50)


                pcl.save(filter, roi_name+str(i)+'.pcd')
                # Separate the table from everything else
                #Astra
                table, objects = do_ransac_plane_segmentation(filter, max_distance = 0.01)#before 0.01

                #realsense
                #table, objects = do_ransac_plane_segmentation(filter, max_distance = 0.0075)#before 0.01

                pcl.save(table, table_name +str(i)+'.pcd')
                pcl.save(objects,objects_name +str(i)+'.pcd')
            flag=False

        for i, cloud in enumerate(rgbd_):
            # Display the table and the object
            pcd = read_point_cloud(objects_name +str(i)+'.pcd')
            write_point_cloud(objects_name +str(i)+'.ply', pcd)
            draw_geometries([pcd])

        print('------------------')
        print('counter:',counter1)

    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camObj=camera()
    main()
