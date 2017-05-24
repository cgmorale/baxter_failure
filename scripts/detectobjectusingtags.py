#!/usr/bin/env python
import sys
import os
import cv2
import numpy as np
import cv2.aruco as aruco


cap = cv2.VideoCapture(0)
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret,th = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
    im2, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_1000)
    parameters =  aruco.DetectorParameters_create()
    markerLength = 4
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners belonging to each id
    corners,ids,rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    aruco.refineDetectedMarkers(gray,board,corners,ids,rejectedImgPoints)
    print(ids)
    cv2.putText(gray, "No Ids", (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv2.LINE_AA) 
    #draw = aruco.drawDetectedMarkers(gray, markers[0],markers[1])
    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()