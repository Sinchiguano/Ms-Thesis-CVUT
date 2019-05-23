#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the MIT License license.

"""

"""

import sys
sys.path.insert(0, '/home/casch/yumi_depends_ws/src/thesis_pkg/yumi_main/scripts/project')

#The order matters
from thesis_library import *
from thesis_class import *
from thesis_function import *

from open3d import uniform_down_sample,registration_icp,TransformationEstimationPointToPlane,TransformationEstimationPointToPoint


voxel_size =0.008

'''registration method'''
def do_dataset(source,target):

    global voxel_size

    print("Downsample the point cloud and get features with FPFH")
    source_down, source_fpfh = do_preprocessing_pcd(source, 0.003)#4mm good for astra
    tmp_source=np.asarray(source_down.points)
    print('shape:',tmp_source.shape)


    print("Downsample the point cloud and get features with FPFH")
    target_down, target_fpfh = do_preprocessing_pcd(target, 0.003)#good tunning
    tmp_target=np.asarray(target_down.points)
    print('shape:',tmp_target.shape)

    return source, target, source_down, target_down, source_fpfh, target_fpfh

def do_preprocessing_pcd(pcd, voxel_size):

    '''Downsample the point cloud'''
    pcd_down = voxel_down_sample(pcd, voxel_size)

    '''Estimate normals'''
    radius_normal = voxel_size * 5
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30))

    '''Fast Point Feature Histograms'''
    radius_feature = voxel_size * 5
    pcd_fpfh = compute_fpfh_feature(pcd_down,KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))

    return pcd_down, pcd_fpfh

def do_ransac_registration(source_down, target_down, source_fpfh, target_fpfh):

    '''RANSAC registration is applied on downsampled point clouds.'''
    #Global registration. This family of algorithms do not require an alignment for initialization.
    #They usually produce less tight alignment results and are used as initialization of the local methods such as ICP.
    #RANSACConvergenceCriteria
    #It defines the maximum number of RANSAC iterations and the maximum number of validation steps.
    #The larger these two numbers are, the more accurate the result is, but also the more time the algorithm takes.
    threshold = 0.003#0.01 * 1.5
    result = registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh,threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),CorrespondenceCheckerBasedOnDistance(threshold)],
            RANSACConvergenceCriteria(800000, 2000))
    return result

def do_icp_registration(source, target, transformation):


    '''Point-to-plane ICP registration is applied on original points'''

    # estimate_normals(source, KDTreeSearchParamHybrid(radius = 0.01, max_nn = 20))
    # estimate_normals(target, KDTreeSearchParamHybrid(radius = 0.01, max_nn = 20))

    threshold = 0.005

    #result = registration_icp(source, target, threshold,transformation,TransformationEstimationPointToPlane())

    # #point to point registration!!!
    result = registration_icp(source, target, threshold, transformation,TransformationEstimationPointToPoint())
    return result

def do_drawing_registration(source, target, transformation):

    source_tmp= copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)
    source_tmp.paint_uniform_color([1, 0.706, 0])
    target_tmp.paint_uniform_color([0, 0.651, 0.929])

    source_tmp.transform(transformation)
    draw_geometries([source_tmp, target_tmp])

def main():
    counter1=0
    counter2=0
    counter3=0
    path_cloud='end_cloud/'


    #path_in='end_cloud_rgbd/'
    path_in='end_cloud_pc/'
    path_out='end_registration/'
    objects_name='objects_name'
    outliars_name='outliars_name'

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        counter1+=1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # LOAD THE POINT CLOUD FROM THE MEMORY
        print("Load a pcd, print it, and render it")
        pcd_ = [read_point_cloud(pcd) for pcd in glob.glob(path_in+'objects_name'+'*pcd')]

        #tmp1=read_point_cloud('pipeline_model/rightFace_m_down.pcd')
        tmp1=pcd_[2]
        tmp2=pcd_[3]
        # #draw_geometries([tmp1,tmp2])


        #DOWNSAMPLE AND COMPUTE FAST POINT FEATURE HISTOGRAM-->PREPROCESSING STEP: DATA MANIPULATION OF THE POINT CLOUD
        source, target, source_down, target_down, source_fpfh, target_fpfh=do_dataset(tmp1,tmp2)
        #draw_geometries([source_down,target_down])

        #exit(0)
        #RANSAC REGISTRATION-->>global registration
        #-------------------
        ransac_output=do_ransac_registration(source_down, target_down, source_fpfh, target_fpfh )
        do_drawing_registration(source, target, ransac_output.transformation)
        print('RANSAC')
        print(ransac_output.transformation)


        #ICP REGISTRATION -->>local registration, point to plane approach
        #-------------------
        # source_down = voxel_down_sample(source, 0.03)
        # target_down = voxel_down_sample(target, 0.03)

        # source_down=np.asarray(source_down.points)
        # target_down=np.asarray(target_down.points)
        # print('shape:',source_down.shape)
        # print('shape:',target_down.shape)

        icp_output = do_icp_registration(source_down, target_down,ransac_output.transformation)
        do_drawing_registration(source, target, icp_output.transformation)
        print('ICP')
        print(icp_output.transformation)



        print("Make a combined point cloud")
        pcd_combined = PointCloud()
        target_copy=copy.deepcopy(target)
        source_copy=copy.deepcopy(source)
        pcd_combined=target_copy
        source_copy.transform(icp_output.transformation)
        pcd_combined +=source_copy
        pcd_combined= voxel_down_sample(pcd_combined, voxel_size = 0.003)
        write_point_cloud(path_out+"multiway_registration.pcd", pcd_combined)
        write_point_cloud(path_out+"multiway_registration.ply", pcd_combined)
        draw_geometries([pcd_combined])

        #cv2.imshow('frame',frame)
        print('counter:',counter1)




    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camObj=camera()
    main()



# draw_geometries([pcd_[0]])
# for i, cloud in enumerate(pcd_):
#     # Display the table and the object
#     draw_geometries([cloud])
#     #flag=False
# '''
# statistical_outlier_removal removes points that are further away from their neighbors compared to the average for the point cloud.
# It takes two input parameters: nb_neighbors: allows to specify how many neighbors are taken into account in order to calculate
# the average distance for a given point. std_ratio: allows to set the threshold level based on the standard deviation of
# the average distances across the point cloud. The lower this number the more aggressive the filter will be.
# '''
# print("Statistical oulier removal")
# cl,ind = statistical_outlier_removal(pcd_[0],nb_neighbors=20, std_ratio=5.0)
# inlier_cloud = select_down_sample(pcd_[0], ind)
# draw_geometries([inlier_cloud])
