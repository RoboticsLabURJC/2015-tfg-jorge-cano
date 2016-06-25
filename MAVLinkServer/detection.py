__author__ = 'JorgeCano'

import math
import jderobot
import numpy as np
import cv2
import threading
from PID import PID

class Detection():
    def __init__(self, sensor):
        self.sensor = sensor
        self.minError=10

        # --- Green ---#
        self.hminG = 45
        self.hmaxG = 95
        self.vminG = 100
        self.vmaxG = 255
        self.sminG = 100
        self.smaxG = 255

        # --- Orange ---#
        self.hminO = 25
        self.hmaxO = 40
        self.vminO = 100
        self.vmaxO = 255
        self.sminO = 100
        self.smaxO = 255

    def detection(self, image, hmin ,hmax, smin, smax, vmin, vmax):

        #------- Thresold Image -------#

        rawImage= image

        ksize = (5,5)
        sigma = 9
        gaussImage = cv2.GaussianBlur(rawImage,ksize,sigma)

        hsvImage = cv2.cvtColor(gaussImage,cv2.COLOR_RGB2HSV)

        matrixMin = np.array([hmin, smin, vmin])
        matrixMax = np.array([hmax, smax, vmax])

        filteredImage = cv2.inRange(hsvImage, matrixMin, matrixMax)

        #------- Detect Object -------#

        minLado = 5
        maxLado = 100

        detImage = rawImage

        contour, hierarchy = cv2.findContours(filteredImage,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        if len(contour) > 0:
            contourdx = -1
            cv2.drawContours(detImage,contour,contourdx,(255,255,0))

        for i in range(len(contour)):
            contarray = contour[i] #Preguntar

            epsilon = 5
            closed = True
            approxCurve = cv2.approxPolyDP(contarray, epsilon, closed)

            rectangle = cv2.boundingRect(approxCurve)
            rectX,rectY,rectW,rectH = rectangle

            if rectW<maxLado and rectW>minLado and rectH<maxLado and rectH>minLado:
                myRectangle = rectangle
            else:
                myRectangle = 0,0,0,0


            myRectX,myRectY,myRectW,myRectH = myRectangle
            myPoint1 = myRectX,myRectY
            myPoint2 = myRectX+myRectW,myRectY+myRectH
            center = myRectX+(myRectW/2),myRectY+(myRectH/2)
            area = myRectW * myRectH

            #cv2.rectangle(detImage,myPoint1,myPoint2,(255,0,0),2)
            cv2.circle(detImage,center,1,(255,0,0),2)

            return center, area


    def execute(self):

        droneImage = self.sensor.getImage()

        centerGreen, areaGreen = self.detection(droneImage, self.hminG ,self.hmaxG, self.sminG, self.smaxG, self.vminG, self.vmaxG)
        centerBlue, areaBlue = self.detection(droneImage, self.hminB ,self.hmaxB, self.sminB, self.smaxB, self.vminB, self.vmaxB)

        centerObjective = len(droneImage)/2 , len(droneImage)/2
        middle = len(droneImage)/2

        cv2.line(droneImage, (0,middle), (2*middle,middle),(255,255,255),)
        cv2.line(droneImage, (middle,0), (middle,2*middle),(255,255,255),)

        xPid = PID(0.008, 0.2, 0)
        xError = centerObjective[0] - centerGreen[0]
        xPid.setError(xError)
        controlX = xPid.calcControl()

        yPid = PID(0.008, 0.2, 0)
        yError = centerObjective[1] - centerGreen[1]
        yPid.setError(yError)
        controlY = yPid.calcControl()

        yawPid = PID(0.1, 0.15, 0)
        yawError = centerGreen[0] - centerBlue[0]
        yawPid.setError(yawError)
        controlYaw = yawPid.calcControl()

        areaObjective = 800
        zPid = PID(0.005, 0.1, 0)
        zError = -(areaObjective - areaGreen)
        zPid.setError(zError)
        controlZ = zPid.calcControl()

        #controlX = 0
        #controlY = 0
        #controlZ = 0
        #controlYaw = 0

        # controlX to veloccY and controlY to veloccX
        self.sensor.sendCMDVel(controlX,controlY,controlZ,controlYaw,0,0)

        #print areaGreen