#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import cv2



def scan_cube():
    """get CubeDefStr from actual Rubics cube, using opencv"""
    # init
    ret = 0
    img = np.zeros((256, 256, 1), dtype = "uint8") # leere "mat"
    cv2.namedWindow("cam preview: [Abort:ESC] [Save:SPACE]")
    # get cam img from preview
    cam = cv2.VideoCapture(0)
    while True:
        ret, img = cam.read()
        cv2.imshow("cam preview: [Abort:ESC] [Save:SPACE]", img)
        if not ret:
            break
        # tastenbedienung
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            cv2.destroyAllWindows() #disable preview window
            print("aborted")
            break
        elif k%256 == 32:
            # SPACE pressed
            #! save/check/process cube-faces
            break
        
    # preprocessing
    # processing
    cam.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # debug override
    retval = 0 
    CubeDefStr = "" 

    return retval, CubeDefStr

if __name__ == "__main__":
    print "manual cube scan started."
    retval, cube = scan_cube()
    print "scanned cube is: %s" %(cube)