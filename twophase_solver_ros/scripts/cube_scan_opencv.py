#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import cv2

from enums import Color # U=0 R=1 F=2 D=3 L=4 B=5
from matplotlib import pyplot as plt # histograms

### config

modus = 1   # vlt über menü auswahl steuern
path = "/home/georg/catkin_ws/src/twophase_solver_ros/images/"
customname = 'example3_'

# get img size
if modus:
    img = cv2.imread(path + customname + 'Color.U.png')
else:
    img = cv2.imread(path + 'example1_' + 'Color.U.png')
#! vlt retval_0 abfangen
height, width, channels = img.shape
imgs = np.array( [np.zeros((height, width, channels), dtype = "uint8")] * 6 )


def save_images(imgs):
    for i in Color:
        filepath = path + customname + str(Color(i)) + '.png'
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
            imgs_pts[i] = cv2.drawKeypoints(imgs[i], keypoints[i], np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) #flags für entsprechende kreisgröße: , cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            
            #! prüfen ob farbzahl stimmt (9 von jeder farbe)
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
        cam = cv2.VideoCapture(2)
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
    
    imgs_hsv = np.zeros((6, height, width, 3), dtype = "uint8")
    imgs_hue = imgs_sat = imgs_val = imgs_gray = np.zeros((6, height, width), dtype = "uint8")
    for i in range(6):
        imgs_hsv[i] = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2HSV_FULL)
        #imgs_gray[i] = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2GRAY)
        #imgs_hue[i], imgs_sat[i], imgs_val[i] = cv2.split(imgs_hsv[i])  #! split-bug: hue kanal wird überschrieben

    cv2.imshow("hue_"+str(0), imgs_hsv[0,:,:,0])
    cv2.imshow("val_"+str(0), imgs_hsv[0,:,:,1])
    cv2.imshow("sat_"+str(0), imgs_hsv[0,:,:,2])
    
    ## entrauschen (median, morph closing, etc)                                                             (kann weg)

    #kernel = np.ones((5,5),np.uint8)
    #imgs_filter_hue = imgs_filter_gray = np.array( [np.zeros((height, width), dtype = "uint8")] * 6 )
    #for i in range (6):
        #imgs_filter_hue[i] = cv2.medianBlur(imgs_hue[i], 7)
        #imgs_filter_gray[i] = cv2.medianBlur(imgs_hue[i], 5)
        #imgs_filter[i] = cv2.morphologyEx(imgs_filter[i], cv2.MORPH_OPEN, kernel)
    # debug override
    #imgs_filter = imgs_hue


    ### processing

    ## vlt bounding box für würfel (roi); stickerflächen finden

    imgs_bin = np.zeros((6, height, width), dtype = "uint8")
    keypoints = np.array( [cv2.KeyPoint()] * 6 )
    
    #- binary image "schwarzes Würfelgitter"
    for i in range(6):

        # adaptive threshhold 
        imgs_bin[i] = cv2.adaptiveThreshold(imgs_hsv[i,:,:,2], 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 81, 5) 

        # morphologische filter
        #imgs_bin[i] = cv2.morphologyEx(imgs_bin[i], cv2.MORPH_OPEN, np.ones((5,5),np.uint8), iterations=1, borderType=cv2.MORPH_CROSS)
        #imgs_bin[i] = cv2.morphologyEx(imgs_bin[i], cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=2, borderType=cv2.MORPH_RECT)
        #cv2.imshow("bincage", imgs_bin[0])

        #? konturen oder andere methode um andere objekte auszuschließen
        #? identifikation des gitters (z.B. zweistufige hirarchie)
        
    
        #? ROI
        # debug override
        #imgs_roi = imgs 
                  
    ## schwerpunkte der sticker finden  (blob detector, hu-momente, ... )
    #! größe des blob detectors vlt relativ zur bounding box
    #? init parameter relativ zu ROI
    # detektieren und nach korrekter keypoint-anzahl prüfen
    keypoints, img_pts = detect_blobs(imgs_bin, 1)
    cv2.imshow('binblob', img_pts[0])
 
    ## schwerpunkt-farben aus hue-kanal mitteln und durch "differenzverfahren" zwischen den farben zuordnen -> zahlenwert als farbe entsprechend enums.py

    # zugriff auf keypoint koordinaten: keypoints[i][0..8].pt[0|1] # 1=row 0=col
    # farbwerte aus originalbild in array speichern. farbwert: imgs_hue[i][row,column]
    # zusammen: color_arr[i][r] = imgs_hue[i][keypoints[i][r].pt[1], keypoints[i][r].pt[0]]    i:facecolors, r:stickercount
    
    #- farbwert des stickerzentrums in array speichern
    color_arr = np.zeros(shape=(6,9,5), dtype = 'int') #! variablentyp overflow bei uint8 
    for i in range(6):
        for r in range(9):
            pt = keypoints[i][r].pt
            x = int(pt[0])
            y = int(pt[1])
            color_arr.itemset((i,r,0), x) # x-Pos
            color_arr.itemset((i,r,1), y) # y-Pos
            color_arr.itemset((i,r,2), int(imgs_hsv.item(i,y,x,0))) # hue 
            color_arr.itemset((i,r,3), int(imgs_hsv.item(i,y,x,1))) # saturation
            color_arr.itemset((i,r,4), int(imgs_hsv.item(i,y,x,2))) # value

    # color_arr nach reihen-koordinaten sortieren    
    for i in range(6):   
        color_arr[i].sort(axis=0)

    # reihen intern nach spalten-koordinaten sortieren    
    for i in range(6):     
        for c in range(3):  
            color_arr[i,c*3:c*3+3] = np.array(sorted(color_arr[i,c*3:c*3+3],key=lambda x: x[1] ) )
            
    
    #! rotation der würfelseiten?
    
    #- danach neue farbwerte zuweisen (0..5 anstatt 0..255)
    value = np.zeros(256, dtype='uint8')
    for i in range(6):
        for r in range(9):
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