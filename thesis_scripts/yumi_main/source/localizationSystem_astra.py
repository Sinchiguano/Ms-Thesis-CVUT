#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2018 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the BSD license.


import sys
sys.path.insert(0, '/home/casch/yumi_depends_ws/src/thesis_pkg/yumi_main/scripts/project')

#The order matters
from thesis_library import *
from thesis_class import *
from open3d import uniform_down_sample,registration_icp,TransformationEstimationPointToPlane,TransformationEstimationPointToPoint
import pcl
import csv
######################################

#from thesis_registration2 import *
model_path='pipeline_model/'
scene_path='pipeline_pcd/'

#folder where i will save my dataset#
#tmp='dataset_astra_CAD/'
tmp='dataset_astra_ownCAD/'
name_file=tmp+'transform_data.csv'


def do_pointcloud(frame,pc,counter):
    global scene_path

    #the following function render a vector in 3d for pcd format in order to use with open3d
    pcd=do_vector3d(pc)

    #we save the pcd in the following formats pcd and ply, and we save a rgb image for visualization purpose
    tmp=scene_path+'pointcloud'+str(counter)
    cv2.imwrite(tmp+'.jpg', frame)

    write_point_cloud(tmp+'base_link'+'.pcd', pcd)
    write_point_cloud(tmp+'base_link'+'.ply', pcd)

    return pcl.load(tmp+'base_link'+'.pcd')

def do_vector3d(pc):

    '''With the help of open3d we render pcd'''
    pc=np.reshape(pc,(np.size(pc[:,:,0]),3))#I took the size of my x coordinates
    #print(pc.shape)
    pc=np.nan_to_num(pc)

    #Pass xyz to Open3D.PointCloud and visualize
    pcd = PointCloud()
    #print('In progress!!!')
    start_timer=time.time()
    pcd.points = Vector3dVector(pc)

    # Flip it, otherwise the pointcloud will be upside down
    #print("Load a pcd point cloud, and flip it!!!")
    #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #Correction

    # #Manual transformation
    # pcd.transform([[-0.0042983,  0.6532477, -0.7571321, 1.20563195],
    #                 [0.9999467, -0.0042983, -0.0093854, 0.11559001],
    #                 [-0.0093854, -0.7571321, -0.6531944, 0.55303995],
    #                 [ 0.,   0.,     0.,     1.,]])
    #with the ROS library# good calibration...
    # pcd.transform([[-0.00921418,  0.67011762, -0.74219773,  1.19737917],
    #     [ 0.99988413, -0.00281966, -0.01495911,  0.11588732],
    #     [-0.01211711, -0.74224957, -0.67001399,  0.51610779],
    #     [ 0. ,         0. ,         0.  ,        1.        ]])
    # pcd.transform([[-0.00726435,  0.71031616, -0.70384529, 1.18878522],
    #                 [ 0.99983368, -0.00661547, -0.01699552,  0.11757787],
    #                 [-0.01672846, -0.70385168, -0.71014996,  0.54026373],
    #                 [ 0.,          0.,          0.,          1.        ]])

    #may_2_2019
    pcd.transform( [[-0.02202798,  0.73070074, -0.68234243,  1.19405833]
                     [ 0.99953549,  0.00171815, -0.03042798,  0.12247143]
                     [-0.02106138, -0.68269574, -0.73039917,  0.53667407]
                     [ 0. ,         0. ,         0. ,         1.        ]])



    return pcd

def do_dataset(source,target):
    '''Preprocessing step'''
    print("Downsample the point cloud and get features with FPFH")
    source_down, source_fpfh = do_preprocessing_pcd(source, 0.004)#in mm
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
    radius_normal = voxel_size * 6
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30))

    '''Fast Point Feature Histograms'''
    radius_feature = voxel_size * 6
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
            RANSACConvergenceCriteria(400000, 1000))
    return result

def do_icp_registration(source, target, transformation):


    '''Point-to-plane ICP registration is applied on original points'''
    estimate_normals(source, KDTreeSearchParamHybrid(radius = 0.01, max_nn = 20))
    estimate_normals(target, KDTreeSearchParamHybrid(radius = 0.01, max_nn = 20))
    threshold = 0.005
    result = registration_icp(source, target, threshold,transformation,TransformationEstimationPointToPlane())
    #point to point registration!!!
    #result = registration_icp(source, target, threshold, transformation,TransformationEstimationPointToPoint())

    return result

def do_drawing_registration(source, target, transformation):

    source_tmp= copy.deepcopy(source)
    target_tmp = copy.deepcopy(target)
    source_tmp.paint_uniform_color([1, 0.706, 0])
    target_tmp.paint_uniform_color([0, 0.651, 0.929])

    source_tmp.transform(transformation)
    draw_geometries([source_tmp, target_tmp])

# Returns Downsampled version of a point cloud. The bigger the leaf size the less information retained
def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
    voxel_filter = point_cloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return voxel_filter.filter()

# Returns only the point cloud information at a specific range of a specific axis
def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
    pass_filter = point_cloud.make_passthrough_filter()
    pass_filter.set_filter_field_name(name_axis)
    pass_filter.set_filter_limits(min_axis, max_axis)
    return pass_filter.filter()

def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):
    '''
    Use RANSAC plane segmentation to separate plane and not plane points
    Returns inliers (plane) and outliers (not plane)
    '''
    segmenter = point_cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(max_distance)

    #obtain inlier indices and model coefficients
    inlier_indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(inlier_indices, negative = False)
    outliers = point_cloud.extract(inlier_indices, negative = True)
    return inliers, outliers

def do_csv_file(tran_rot,counter1,computation_time):
    global name_file
    aux_angles=do_rotation_matrix_to_euler_angles(tran_rot[:-1,:-1])
    tmp_list=list(tran_rot[:-1,3])
    #q = tf.transformations.quaternion_from_matrix() 
    from pyquaternion import Quaternion
    a= Quaternion(matrix=tran_rot)
    # print(a)
    # print("{} + {}i + {}j + {}k".format(a[0], a[1], a[2], a[3]))
    quat=[a[0], a[1], a[2], a[3]]
    with open(name_file, 'a') as csvfile:# a means append
        filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        #filewriter.writerow(aux_angles+tmp_list+quat)
        filewriter.writerow(aux_angles+tmp_list+computation_time)

# Calculates rotation matrix to euler angles
def do_rotation_matrix_to_euler_angles(R) :
    #inspire by the following source
    #https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return [x* 180 / math.pi, y* 180 / math.pi, z* 180 / math.pi]

def main():
    counter1=0
    counter2=0
    counter3=0
    global tmp

    import sys
    print "This is the name of the script: ", sys.argv[0]
    rate = rospy.Rate(10) # 10hz

    flag=False

    while not rospy.is_shutdown():
        # Capture 2D-data
        frame=camObj.get_image()

        # Capture depth data
        depth=camObj.get_depth()

        # Capture 3D-data
        pc=camObj.get_point_cloud()

        if pc is None:
            print('no PointCloud2!!!')
            continue
        elif frame is None:
            print('no Frame!!!')
            continue
        elif depth is None:
            print('no depth image!!!')
            continue

        #Enter to process a depth image and generate the 6D pose estimation
        command=cv2.waitKey(1) & 0xFF
        print('Ready to take scene data!!!')

        if command == ord('e'):
            counter1+=1
            """Get a scene cloud in the world coordinate system"""
            pcd=do_pointcloud(frame,pc,counter2)
            cloud=copy.deepcopy(pcd)

            #######
            #for testing purposes
            pcl.save(pcd,tmp+str(counter1)+'cloud'+'.pcd')
            pcd = read_point_cloud(tmp+str(counter1)+'cloud'+'.pcd')
            write_point_cloud(tmp+str(counter1)+'cloud'+'.ply', pcd)
            cv2.imwrite(tmp+str(counter1)+'img'+'.jpg', frame)
            #draw_geometries([pcd])

            #######
            # print('	//Ready to collect 3D-data')
            # print "\n============ Press `Enter` to start..."
            # raw_input()
            # counter2+=1

            # Mask out point cloud in order to get only information of our region of interest, as we don't care about the other parts
            #--------------------------------------------------------
            # Threshold when working with the astra
            filter = do_passthrough_filter(point_cloud = cloud,name_axis = 'x', min_axis = 0.25, max_axis = 0.75)
            filter = do_passthrough_filter(point_cloud = filter,name_axis = 'y', min_axis = -0.10, max_axis = 0.50)
            pcl.save(filter,scene_path +'filter_objects_'+str(counter2)+'.pcd' )
            print('filter done!')

            # Segmentation process in order to separate the object from table
            #----------------------------------------------------------------
            # Astra
            table, objects = do_ransac_plane_segmentation(filter, max_distance = 0.01)
            pcl.save(objects,scene_path +'objects_'+str(counter2)+'.pcd' )
            print('segmentation done!')

            pcd = read_point_cloud(scene_path+'objects_'+str(counter2)+'.pcd')
            write_point_cloud(scene_path +'objects_'+str(counter2)+'.ply', pcd)
            print('change done')

            #The source cloud is my CAD model that it is already in the world coordinate system
            #source=read_point_cloud(model_path+'front_face_m_down.pcd')
            source=read_point_cloud(model_path+'objects_0_render_m.ply')

            #The target cloud is a scene image, it is already mapped into the world coordinate system (T: World -> Camera)
            target=read_point_cloud(scene_path+'objects_'+str(counter2)+'.pcd')
            draw_geometries([source,target])
            print('Uploading done!!!')
            
            #DOWNSAMPLE AND COMPUTE FAST POINT FEATURE HISTOGRAM-->PREPROCESSING STEP: DATA MANIPULATION OF THE POINT CLOUD
            source, target, source_down, target_down, source_fpfh, target_fpfh=do_dataset(source,target)
            draw_geometries([source_down,target_down])

            import time
            start = time.time()

            #RANSAC REGISTRATION-->>global registration
            #-------------------
            ransac_output=do_ransac_registration(source_down, target_down, source_fpfh, target_fpfh )
            #ICP REGISTRATION -->>local registration, point to plane approach
            #-------------------
            icp_output = do_icp_registration(source, target,ransac_output.transformation)
            end = time.time()
            print(end - start)

            print(icp_output)
            do_csv_file(icp_output.transformation,counter1,[end - start])
            do_drawing_registration(source, target, ransac_output.transformation)
            do_drawing_registration(source, target, icp_output.transformation)
            print('ICP')


        cv2.imshow('frame',frame)

    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camObj=camera()
    main()
