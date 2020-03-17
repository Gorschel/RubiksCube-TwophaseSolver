#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import cv2

from enums import Color # U=0 R=1 F=2 D=3 L=4 B=5
modus = 1   # vlt über menü auswahl steuern
path = "/home/georg/catkin_ws/src/twophase_solver_ros/images/"
customname = 'example2_'

img = cv2.imread(path + customname + 'Color.U.png')
height, width, channels = img.shape
imgs = np.array( [np.zeros((height, width, channels), dtype = "uint8")] * 6 )


def save_images(imgs):
    for i in Color:
        filepath = path + str(Color(i)) + '.png'
        cv2.imwrite(filepath, imgs[i])
        print "image saved: " + filepath

def load_images():
    for i in Color:
        filepath = path + customname + str(Color(i)) + '.png'
        imgs[i] = cv2.imread(filepath)
        print "image loaded: " + filepath
    return imgs

def generateDefParams():
    'create default parameter set'
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 256
    params.thresholdStep = 37
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 10000
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.maxConvexity = 1.0
    # mglw verbugt
    params.filterByColor = False
    params.blobColor = 0 #0..255
    return params

def detect_blobs(imgs, trys):
    keypoints = np.array( [cv2.KeyPoint()] * 6 )
    imgs_pts = np.array( [np.zeros((height, width, channels), dtype = "uint8")] * 6 )
    cyc = False

    for i in range(6):
        params = generateDefParams() # create default parameter set
        cyc = True
        while cyc:
            detector = cv2.SimpleBlobDetector_create(params) # create detector
            keypoints[i] = detector.detect(imgs[i]) # detect
            imgs_pts[i] = cv2.drawKeypoints(imgs[i], keypoints[i], np.array([]), (0,0,255)) #flags für entsprechende kreisgröße: , cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            if np.size(keypoints[i]) == 9: # erfolg
                cyc = False
            elif np.size(keypoints[i]) < 9 and trys>0: # zu wenige
                params.minArea -= 10
                params.minConvexity -= 0.01
                trys -= 1
            elif np.size(keypoints[i]) > 9 and trys>0: # zu viele
                params.minArea += 10
                params.minConvexity += 0.01
                trys -= 1
            elif trys == 0:
                break
            # anzahl alleine reicht nicht aus

    return keypoints, imgs_pts

def scan_cube():
    """get CubeDefStr from actual Rubics cube, using opencv"""


    ### init

    ret = 0
    win_name = "init"  # vlt named window für menü hier erstellen


    ### vlt menü

    # plot text in blank image
    # actions like save imgs, start scan, etc


    ### get cube face images

    if modus == 0:      # bilder aufnehmen
        cam = cv2.VideoCapture(0)
        width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        channels = 3
        imgs = np.array( [np.zeros((height, width, channels), dtype = "uint8")] * 6 )

        cv2.namedWindow(win_name)
        for i in Color:
            # update window name
            win_name_new = "cam preview for Face" + str(Color(i)) + ": [ESC:abort, SPACE:save]"
            cv2.setWindowTitle(win_name, win_name_new)   
            # preview camera feed
            while True:
                ret, frame = cam.read()
                cv2.imshow(win_name, frame)
                if not ret: 
                    retval = 0 
                    break
                k = cv2.waitKey(1)
                if k%256 == 27:
                    # ESC pressed
                    cv2.destroyAllWindows() #disable preview window
                    print("aborted")
                    retval = 1
                    break
                elif k%256 == 32:
                    # SPACE pressed
                    imgs[i] = frame
                    retval = 2
                    break
            if retval < 2: break    
            win_name = win_name_new
        cv2.destroyAllWindows
        cam.release()
        save_images(imgs)
    elif modus == 1:    # bilder laden
        imgs = load_images()
        # cv2.imshow('test', imgs[0]) # vorschau if erfolg


    ### preprocessing

    height, width, channels = imgs[0].shape

    ## farbraum trafo / split
    
    imgs_hsv = np.array( [np.zeros((height, width, 3), dtype = "uint8")] * 6 )
    imgs_hue = imgs_sat = imgs_val = imgs_gray = np.array( [np.zeros((height, width), dtype = "uint8")] * 6 )
    for i in range(6):
        imgs_hsv[i] = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2HSV_FULL)
        imgs_gray[i] = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2GRAY)
        imgs_hue[i], imgs_sat[i], imgs_val[i] = cv2.split(imgs_hsv[i])

    ## entrauschen (median, morph closing, etc)
    kernel = np.ones((5,5),np.uint8)
    imgs_filter_hue = imgs_filter_gray = np.array( [np.zeros((height, width), dtype = "uint8")] * 6 )
    for i in range (6):
        imgs_filter_hue[i] = cv2.medianBlur(imgs_hue[i], 7)
        imgs_filter_gray[i] = cv2.medianBlur(imgs_hue[i], 5)
        #imgs_filter[i] = cv2.morphologyEx(imgs_filter[i], cv2.MORPH_OPEN, kernel)
    # debug override
    #imgs_filter = imgs_hue


    ### processing

    ## bounding box für würfel (roi); stickerflächen finden

    imgs_bin = imgs_bin_val = imgs_bin_sat  = imgs_bin_gray = np.array( [np.zeros((height, width), dtype = "uint8")] * 6 )
    for i in range(6):
        #- binary image "schwarzes Würfelgitter"
        ret, imgs_bin_val[i] = cv2.threshold(imgs_val[i], 70, 255, cv2.THRESH_BINARY_INV)
        ret, imgs_bin_sat[i] = cv2.threshold(imgs_sat[i], 70, 255, cv2.THRESH_BINARY_INV)
        ret, imgs_bin_gray[i] = cv2.threshold(imgs_filter_gray[i], 150, 255, cv2.THRESH_BINARY_INV)
        imgs_bin[i] = cv2.bitwise_not( cv2.bitwise_and(imgs_bin_sat[i], imgs_bin_val[i]), imgs_bin_gray[i])

        #- konturen oder andere methode um andere objekte auszuschließen

        #- identifikation des gitters (z.B. zweistufige hirarchie)
        
        #- Sticker im Gitter finden

        #- ROI

        # plot
        cv2.imshow("bincage_"+str(i), imgs_bin[i])
    
    #- debug override
    imgs_roi = imgs 

    ## schwerpunkte der sticker finden  (blob detector, hu-momente, ... )
    #! größe des blob detectors vlt relativ zur bounding box
    #? init parameter relativ zu ROI
    # detektieren und nach korrekter keypoint-anzahl prüfen
    #{ example1: es werden nicht alle felder entdeckt. 1 und 3 problematisch}
    
    #keypoints, imgs_pts = detect_blobs(imgs_filter_hue, 10)
    #cv2.imshow('blobs debug 1', imgs_pts[1])
    #cv2.imshow('blobs debug 3', imgs_pts[3])
 
    ## schwerpunkt-farben mitteln und durch "differenzverfahren" zwischen den farben zuordnen -> zahlenwert als farbe entsprechend enums.py

    # zugriff auf keypoint koordinaten: keypoints[i][0..8].pt[0|1] 
    # farbwerte aus originalbild in array speichern. farbwert: imgs_hue[i][row,column]
    #- zusammen: color_arr[i][r] = imgs_hue[i][keypoints[i][r].pt[0], keypoints[i][r].pt[1]]    i:facecolors, r:stickercount

    #! wäre vmtl noch unsortiert
    #- farbdifferenzverfahren über alle farbpunkte in color_arr

    #- danach neue farbwerte zuweisen (0..5 anstatt 0..255)

    #! prüfen ob farbzahl stimmt (9 von jeder farbe)

    #- farbwert des stickerzentrums in array speichern
    color_arr = np.zeros(shape=(6,9)) 
    for i in range(6):
        for r in range(9):
            #color_arr[i][r] = int( imgs_hue[i][int(keypoints[i][r].pt[0]), int(keypoints[i][r].pt[1])] )
            pass

    ## gescannte würfelseiten in validen CubeDefString umwandeln
    #? rotation wichtig


    ### deinit
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    ### debug override
    retval = 0 
    CubeDefStr = "override. finished" 


    return retval, CubeDefStr

if __name__ == "__main__":
    print "manual cube scan started."
    retval, cube = scan_cube()
    print "scan result: %s" %(cube)