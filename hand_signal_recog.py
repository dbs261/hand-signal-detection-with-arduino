# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 18:19:33 2020

@author: Dhanush
"""

import cv2
import numpy as np
import serial #PySerial

#kernel for erosion operation
kernel=np.ones((3,3),dtype=np.uint8)

#start serial communication with arduino
ser=serial.Serial('COM4',baudrate=9600)

#Start video capture
cap=cv2.VideoCapture(0)

#a while loop to keep capturing frames from your web cam 
while True:
    # Read the frame, flip it to your convenience, and convert a copy
    # of it to gray scale 
    ret,_frame=cap.read()
    
    _frame=cv2.flip(_frame,1)
    frame=cv2.cvtColor(_frame,cv2.COLOR_BGR2GRAY)
    
    #thresholding the image so as to separate your hand from a BLACK background
    ret,mask=cv2.threshold(frame,120,255,cv2.THRESH_BINARY)
    
    #Once you position your hand in a rectangular section at the bottom of your screen,
    #this separates the image of your hand from the frame and positions it upright.
    hand=np.copy(mask[280:480,130:400])
    hand=cv2.flip(hand,0)
    
    #Highlighting the place to put you hand in the mask and coloured frame
    cv2.rectangle(mask,(130,280),(400,480),(255,255,255),1)
    cv2.rectangle(_frame,(130,280),(400,480),(255,255,255),1)
    
    # Slightly erodes the figure of your hand
    hand=cv2.erode(hand,kernel,iterations=2)
    
    #finds the coordinates of all the contours of the image of your hand
    contours, _ = cv2.findContours(hand, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #find the set of contours with maximum area, i.e, your hand
    try:
        cnt=max(contours,key=lambda x:cv2.contourArea(x))
    except:
        continue
    
    #converting the hand frame to BGR colour scheme so that the contours drawn
    #later on will be coloured (else, they will appear black or white)
    hand=cv2.cvtColor(hand,cv2.COLOR_GRAY2BGR)
    
    #this function groups together nearby contour points and stores it in approx
    approx = cv2.approxPolyDP(cnt, 0.020 * cv2.arcLength(cnt, True), True) 
    
    # draws boundary of contours. 
    cv2.drawContours(hand, [approx], 0, (0, 0, 255), 3) 

    #to find the bottom most contour point(ends of the wrist)
    for i,j in enumerate(approx):
        x,y=j[0]
        if y==199:
            break
        
    #to get the contour points in anticlock wise order from the bottom
    h=np.array(list(approx[i+1:])+list(approx[:i+1]))

    #This part is explained better in the documentation
    count=0
    if 5000.0<cv2.contourArea(cnt)<18000.0:
        for i in range(0,len(h)-2,2):
            x=h[i][0]
            y=h[i+1][0]
            z=h[i+2][0]
            c=np.linalg.norm(y-x)
            b=np.linalg.norm(y-z)
            a=np.linalg.norm(z-x)

            theta=np.arccos((b*b+c*c-a*a)/(2*b*c))*180/(3.141592653589)
            if (0.0<theta<70.0) and b>40.0 and c>40.0:
                count+=1
    
    #For the counter frame (numbers)
    blank=np.zeros((400,400),dtype=np.uint8)
    cv2.putText(blank,'{}'.format(count),(100,300),cv2.FONT_HERSHEY_SCRIPT_COMPLEX,3,(255,0,0),2,cv2.LINE_AA)

    #Sending the value of the speed through serial, where it will be accepted by the arduino through bluetooth
    ser.write("{:.1f}".format(count*0.2).encode())
    print("{:.1f}".format(count*0.2))
    
    #display all the screens
    cv2.imshow('number',blank)
#     cv2.imshow('GRAY',frame)
    cv2.imshow('frame',_frame)
    cv2.imshow('mask',mask)
    cv2.imshow('hand',hand)

    # To exit press q
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
#end of while loop
#stops video, destroys all screens
cap.release()
cv2.destroyAllWindows()

#stops serial communication with arduino
ser.close()
