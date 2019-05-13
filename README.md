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


## Citation
Please cite the work in your publications if it helps your research:
   
    @inproceedings{casch2019cv,
      author = {Cesar Sinchiguano},
      booktitle = {Master_Thesis},
      title = {Part localization for robotic manipulation},
      year = {2019}
      }

By: Casch...

