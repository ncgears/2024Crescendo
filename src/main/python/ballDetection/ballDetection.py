from re import A
import cv2
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import threading
from networktables import NetworkTables
import sys

cond = threading.Condition()
notified = [False]
ip='127.0.0.1'
# ip='roborio-1918-frc.local'
tableName= "VisionInfo"
print(sys.argv,'sys')
def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server=ip)
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0] :
        cond.wait()

table = NetworkTables.getTable(tableName)
# table = table.getTable(tableName)

print("Connected!")


wanted_color = 'both'
# wanted_color = table.getString("desiredColor","none")

cv2.namedWindow("window")
cv2.namedWindow('real')
vs = cv2.VideoCapture(0)

blue_lower = np.array([90, 60, 50], np.uint8)
blue_upper = np.array([150, 255, 255], np.uint8)
lower_red = np.array([0, 130, 50]) 
upper_red = np.array([50, 255, 255])
lower_red_to_delete_blue = np.array([150, 130, 50])  
upper_red_to_delete_blue = np.array([255, 255, 255])
bothlow = np.array([0, 130, 60])  
bothup = np.array([180, 255, 255]) 


def findball(mask, color='both'):
    global contour_dict
    mask = cv2.erode(mask, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=5)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_LIST,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    
    for contour in cnts:
        approx = cv2.approxPolyDP(
            contour, 0.005*cv2.arcLength(contour, True), True)

        if (len(approx) > 10):
            a = contour_dict[color]
            a.append(contour)
            contour_dict[color] = a

while True:
    _, image = vs.read()
    image = imutils.resize(image, width=500)
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    maskblue = cv2.inRange(hsv, blue_lower, blue_upper)
    maskred1 = cv2.inRange(hsv, lower_red, upper_red)
    maskred2 = cv2.inRange(hsv, lower_red_to_delete_blue,
                           upper_red_to_delete_blue)
    maskred = cv2.bitwise_or(maskred1, maskred2)
    contour_dict = {'red': [], 'blue': [], 'both': []}
    ball=[]
    ball_info_dict = {'red': [], 'blue': [], 'both': []}
    findball(maskred, 'red')
    findball(maskblue, 'blue')
    mask = cv2.bitwise_or(maskred, maskblue)
    masked = cv2.bitwise_and(image, image, mask=mask)
    for color, ctnlist in contour_dict.items():
        for c in ctnlist:
            (x, y), radius = cv2.minEnclosingCircle(c)
            x = int(x)
            y = int(y)
            area = cv2.contourArea(c)
            l, o, w, h = cv2.boundingRect(c)
            rect_area = w*h
            extent = float(area)/rect_area

            if extent > 0.6 and area > 1000:
                if color!='both':
                    a= ball_info_dict[color] 
                    a.append([x, y, area,color])
                    ball_info_dict[color] =a
                    ball.append([x, y, area])
                cv2.circle(masked, (int(x), int(y)),
                           int(radius), (255, 255, 255), 5)
                if color:
                    cv2.putText(image, text=color, org=(x, y),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0),
                                thickness=2, lineType=cv2.LINE_AA)
    ball_info_dict['both'] = ball_info_dict['red']+ball_info_dict['blue']
    print(ball_info_dict)
    def sort(e):
        #sortby
        # 0 = x , 1=y , 2 =area
        return e[0]
    # if ball != []:
    #     ball.sort(reverse=True, key=sort)
    #     print(table.getKeys())
    #     for i in range(len(ball)):
    #         table.putNumberArray('target{}x'.format(i),ball[i][0])
    #         table.putNumberArray('target{}y'.format(i),ball[i][1])
    #         table.putNumberArray('target{}size'.format(i),ball[i][2])
    #         table.putNumberArray('target{}color'.format(i),ball[i][2])
        
    #         #need to be entries called target[i]x, target[i]y, target[i]color
    #     try:
    #         for i in range(len(ball) or None):
    #             cv2.putText(image, text=str(i+1), org=(ball[i][0], ball[i][1]+50),
    #                         fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0),
    #                         thickness=2, lineType=cv2.LINE_AA)
    #     except:
    #         pass
    # else:
    #     for key in table.getKeys():
    #         table.putNumberArray(key,[])
    ball= ball_info_dict[wanted_color]
    if ball != [] and wanted_color!="none":
        ball.sort(reverse=True, key=sort)
        print(ball)
        for i in range(min(len(ball),3)):
           
            table.putValue('target{}x'.format(i+1),ball[i][0])
            table.putValue('target{}y'.format(i+1),ball[i][1])
            table.putValue('target{}size'.format(i+1),ball[i][2])
            table.putString('target{}color'.format(i+1),ball[i][3])
      
            # table.putNumber('target{}x'.format(i+1),ball[i][0])
            # table.putNumber('target{}y'.format(i+1),ball[i][1])
            # table.putNumber('target{}size'.format(i+1),ball[i][2])
            # table.putString('target{}color'.format(i+1),ball[i][3])
            # print('updateball{}'.format(i+1))
        
            #need to be entries called target[i]x, target[i]y, target[i]color
        try:
            for i in range(len(ball) or None):
                cv2.putText(image, text=str(i+1), org=(ball[i][0], ball[i][1]+50),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0),
                            thickness=2, lineType=cv2.LINE_AA)
        except:
            pass
        for i in range(len(ball),3) :
            table.putValue('target{}x'.format(i+1),-1)
            table.putValue('target{}y'.format(i+1),-1)
            table.putValue('target{}size'.format(i+1),-1)
            table.putValue('target{}color'.format(i+1),"none")

    else:
        for i in range(3) :
            table.putValue('target{}x'.format(i+1),-1)
            table.putValue('target{}y'.format(i+1),-1)
            table.putValue('target{}size'.format(i+1),-1)
            table.putValue('target{}color'.format(i+1),"none")


    cv2.imshow('window',  masked)
    cv2.imshow('real', image)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
