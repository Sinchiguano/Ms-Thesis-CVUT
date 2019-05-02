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



def create_dataset_images(frame,counter):
    global path_images

    tmp='img_astra-'+str(counter)
    cv2.imwrite(tmp+'.jpg', frame)

def main():
    counter1=0
    counter2=0


    import sys
    print "This is the name of the script: ", sys.argv[0]

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        counter1+=1

        # Capture 2D-data
        frame=camObj.get_image()

        if frame is None:
            print('no Frame!!!')
            continue

        command=cv2.waitKey(1) & 0xFF
        print('ready to start collecting 3D-data! ')

        #Collect images for calibration
        if command == ord('p'):
            counter2+=001
            create_dataset_images(frame,counter2)
            print('next!')
            print('counter: {}'.format(counter2))
            time.sleep(1)
        if command == ord('q'):
            break
        try:
            # 2D image points
            # To handle the corners array more easily, we can reshape it as follows
            ret, corners = cv2.findChessboardCorners(frame, (8,9))#column and rows
            corners=corners.reshape(-1,2)#undefied number of rows
            cv2.drawChessboardCorners(frame, (8,9), corners, ret)
            if not ret:
                print('\nPlease, locate well the calibration target!!!')
                continue
        except Exception as ex:
            print('\nStatus of findChessboardCorners: {}'.format(ret))
            print('Please, locate well the calibration target!!!')
            print(ex)
            print('-------------------------------------------------')
            continue

        cv2.imshow('frame',frame)

    # When everything done, release the capture
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camObj=camera()
    main()
