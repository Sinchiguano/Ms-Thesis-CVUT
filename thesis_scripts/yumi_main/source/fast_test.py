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
from thesis_library import *

# from yumi_class import MoveGroup,all_close,measurements
from yumipy import YuMiRobot
from yumipy import YuMiState
from autolab_core import RigidTransform


# moves_deg = np.array([[-118.83, -106.99, 0.21, 68.64, -16.03, -81.35, 42.61],
#                     [-122.66, -99.11, -12.67, 81.03, -12.68, -96.27, 39.75],
#                     [-118.22, -104.42, 14.18, 31.72, -25.17, -44.6, 41.04],
#                     [-118.78, -95.0, -18.42, 106.49, -7.06, -126.19, 35.77],
#                     [-108.27, -106.58, 1.2, 69.36, -6.28, -90.27, 37.11],
#                     [-112.79, -104.75, -1.72, 69.43, -8.85, -88.0, 38.16],
#                     [-112.79, -104.75, -1.72, 69.43, -8.85, -88.0, 38.16],
#                     [-121.92, -101.36, -24.07, 129.62, -17.69, -145.72, 40.5],
#                     [-116.35, -105.67, -0.9, 68.06, -12.88, -83.43, 40.57],
#                     [-123.22, -103.88, -5.89, 73.88, -17.0, -85.8, 42.66],
#                     [-115.78, -99.11, 8.09, 17.54, -19.96, -35.91, 35.19],
#                     [-130.46, -92.83, -26.13, 97.0, -14.9, -111.37, 40.55],
#                     [-122.44, -90.82, -26.37, 120.9, -8.51, -140.31, 35.86],
#                     [-130.67, -99.9, -15.62, 83.18, -19.65, -93.41, 43.71],
#                     [-122.44, -90.82, -26.37, 120.9, -8.51, -140.31, 35.86],
#                     [-128.35, -96.28, -38.09, 136.85, -22.77, -153.47, 40.52],
#                     [-120.01, -113.2, 3.23, 48.81, -24.55, -59.58, 47.03],
#                     [-112.39, -112.09, 4.86, 35.69, -18.51, -52.05, 43.47],
#                     [-103.02, -108.17, -0.79, -5.72, -7.51, -19.48, 36.51],
#                     [-129.67, -96.58, -38.35, 134.75, -23.36, -150.59, 40.99],
#                     [-97.33, -129.77, 27.04, 49.68, -22.46, -59.93, 57.7],
#                     [-120.22, -103.22, -9.72, 40.74, -16.0, -57.68, 40.86]])

#set tool according to a calibr
tool_cesar_cal = RigidTransform(np.array([[0, 0, 1],
                                          [0, -1, 0],
                                          [1, 0, 0]]), np.array([0, 0.0, 0.0]))


#Home position
home_left=[1.25, -129.84, 29.98, -0.76, 40.78, -5.78, 136.29]


g_timestamp_last_move = 0
g_index_last_move = 0

def move(yumi_robot):
    import time
    global g_index_last_move
    global g_timestamp_last_move

    # if (time.time() - g_timestamp_last_move) < 1:
    #     return

    #Object that encapsulates a yumi arm joint angle configuration.
    moves = [YuMiState(p) for p in moves_deg]

    g_index_last_move = (g_index_last_move + 1) % len(moves)

    yumi_robot.left.goto_state(moves[g_index_last_move],wait_for_res=False)
    g_timestamp_last_move = time.time()


def pose_to_tf(br,pose_translation,pose_quaternion):

    """input in m and rad"""

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = 'yumi_tcp'
    t.transform.translation.x = pose_translation[0]
    t.transform.translation.y = pose_translation[1]
    t.transform.translation.z = pose_translation[2]
    t.transform.rotation.w = pose_quaternion[0]
    t.transform.rotation.x = pose_quaternion[1]
    t.transform.rotation.y = pose_quaternion[2]
    t.transform.rotation.z = pose_quaternion[3]
    br.sendTransform(t)

def main():
    global home_left
    rospy.init_node('tcp_tf', anonymous=True)
    br = tf2_ros.TransformBroadcaster()

    # starting the robot interface
    y = YuMiRobot(arm_type='remote')

    global tool_cesar_cal
    y.left.set_tool(tool_cesar_cal)
    rate = rospy.Rate(10)

    # state = y.left.get_state(raw_res=False)
    # print(state)
    #home_left=YuMiState(home_left)
    #y.left.goto_state(home_left,wait_for_res=False)
    # pose = y.left.get_pose(raw_res=False)
    # print(pose.translation)
    # print(pose.rotation)
    # time.sleep(3)
    # state1=[-134.07, -50.55, -4.95, -64.58, -58.61, 95.96, 134.53]
    # state1=YuMiState(state1)
    # y.left.goto_state(state1,wait_for_res=False)
    #exit(0)

    while (True):

        pose = y.left.get_pose(raw_res=False)

        print('translation {}'.format(pose.translation))
        print('quaternion {}'.format(pose.quaternion))
        print('rotation matrix \n{}'.format(pose.rotation))
        print(pose)
        print('moving!!!')

        pose.translation+=0.05
        y.left.goto_pose_linear_path(pose, wait_for_res=True, traj_len=10, eef_delta=0.01, jump_thresh=0.0)
        #pose.quaternion= [-0.0640144  -0.702158   -0.07501688  0.70515868]

        #move(y)
        #pose_to_tf(br,pose.translation,pose.quaternion)


        # we should expect to go through the loop 10 times per second
        rate.sleep()
if __name__ == '__main__':
    main()
