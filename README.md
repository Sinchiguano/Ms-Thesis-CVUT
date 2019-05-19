# Ms-Thesis-CVUT
# Part localization for robotic manipulation
By [Cesar Sinchiguano](https://github.com/Sinchiguano).

<p align="left">
<img src="https://github.com/Sinchiguano/Ms-Thesis-CVUT/blob/master/tmp/yumi1.gif", width="430">
<img src="https://github.com/Sinchiguano/Ms-Thesis-CVUT/blob/master/tmp/yumi2.gif", width="430">
</p>

## Abstract

The new generation of the collaborative robots allows the use of small robot arms working with human workers, e.g. the YuMi robot, a dual 7-DOF robot arms designed for precise manipulation of small objects. For the further acceptance of such a robot in the industry, some methods and sensors systems have to be developed to allow them to perform a task such as grasping a specific object. If the robot wants to grasp an object, it has to localize the object relative to itself. This is a task of object recognition in computer vision, the art of localizing predefined objects in image sensor data. This master thesis presents a pipeline for object recognition of a single isolated model in point cloud.
The system uses point cloud data rendered from a 3D CAD model and describes its characteristics using local feature descriptors. These are then matched with the descriptors of the point cloud data from the scene to find the 6-DoF pose of the model in the robot coordinate frame. This initial pose estimation is then refined by a registration method such as ICP. A robot camera calibration is performed also. The contributions of this thesis are as follows:
The system uses FPFH (Fast Point Feature Histogram) for describing the local region and a hypothesize-and-test paradigm, e.g. RANSAC in the matching process.
In contrast to several approaches those whose rely on Point Pair Features as feature descriptors and a geometry hashing, e.g. voting-scheme as the matching process.

## Requirements

In order to use the yumi_main package where the Eye_To_Hand calibration and the 3D object pose estimation system are developed, the following dependencies have to bo met:

 * Linux Ubuntu 16.04 
 * [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation)
 * Python 2.7.6, 3.4.0, 3.5.2
 * pcl 1.7.0
 * Cython <= 0.25.2
 * [YuMi Python Interface](https://github.com/BerkeleyAutomation/yumipy)

### Step-by-Step: Eye-To-Hand Calibration

A typical use case consists of the following steps (here using ROS kinetic):

* Interface the robot with a terminal computer by executing the following launch file: 
    roslaunch yumipy yumi_arms.launch 

* Execute publishingTF.py in order to command the robot movement.

* Execute the following python file:

    camera_robotCAL_real.py or camera_robotCAL_astra.py, depend on the camera to be calibrated. A transform from the calibration target to the camera starts publishing into the ROS network. 

* Finally, execute the listeningTF.py in order to retrieve the transformation of the camera relative to the robot frame.

* If you wish, run Rviz in order to see the tf tree.


### Step-by-Step: 3D Object Pose Estimation System

A typical use case consists of the following steps (here using ROS kinetic):

* Execute the camera driver.

* Execute demo.py 




## Citation
Please cite the work in your publications if it helps your research:
   
    @inproceedings{casch2019cv,
      author = {Cesar Sinchiguano},
      booktitle = {Master_Thesis},
      title = {Part localization for robotic manipulation},
      year = {2019}
      }

By: Casch...

