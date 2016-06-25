__author__ = 'JorgeCano'

import sys, traceback, Ice
import jderobot, time
import random
import math
import numpy as np
import cv2

from Pose3D import Pose3DI
from CMDVel import CMDVelI
from Extra import ExtraI
from PID import PID

import threading


##### Globals #####

lock = threading.Lock()

# --- Trajectory ---#

MissionHeight = 10      #metres
StartWaypoint = [0,0,MissionHeight]
ScanDistance = 5        #metres
SpinsNumber = 3
ReachedDist = 0.5       #metres

# --- Camera ---#

# CameraHeight = int
# CameraWidth = int

# --- Target ---#

minTargetLado = 20
maxTargetLado = 1000

# Green
hminG = 45
hmaxG = 90
vminG = 80
vmaxG = 255
sminG = 100
smaxG = 255

# Orange
hminO = 10
hmaxO = 40
vminO = 100
vmaxO = 255
sminO = 110
smaxO = 255

# --- Hover control ---#

HorP = 0.0008
HorD = 0.02
HorI = 0

VerP = 0.0008
VerD = 0.02
VerI = 0

###################

def rxPose3D(Pose3D):

    status = 0
    ic = None
    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Pose3D:default -p 9998")
        datos = jderobot.Pose3DPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(0.02)
            data = datos.getPose3DData()
            Pose3D.setPose3DData(data)
            #print Pose3D

    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def txPose3DWP2Server(Pose3D):

    status = 0
    ic = None
    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Pose3D:default -p 9994")
        datos = jderobot.Pose3DPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(0.02)

            Pose3D2send = Pose3D.getPose3DData()
            datos.setPose3DData(Pose3D2send)

            # print Pose3D

    except:
        traceback.print_exc()
        status = 1

    if ic:

        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def txCMDVel2Server(CMDVel):

    status = 0
    ic = None

    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("CMDVel:default -p 9997")
        datos = jderobot.CMDVelPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(0.05)
            CMDVel2send = CMDVel.getCMDVelData()
            datos.setCMDVelData(CMDVel2send)
            #print CMDVel2send


    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def txExtra2Server(Extra): #DO NOT REFRESHING NOTHING !!!!!

    status = 0
    ic = None

    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Extra:default -p 9995")
        datos = jderobot.ArDroneExtraPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(0.05)
            Extra2send = Extra
            #datos.setExtraData(Extra2send)
            #print CMDVel2send

            #DOING NOTHING !!!!!

    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def rxCamera():

    status = 0
    ic = None

    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Camera:default -p 9999")
        datos = jderobot.CameraPrx.checkedCast(base)
        # print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        # global PH_Camera
        # PH_Camera = datos.startCameraStreaming()

        while True:
            time.sleep(0.5)
            global PH_Camera
            global CameraHeight
            global CameraWidth
            cameraImage = datos.getImageData("RGB8")
            CameraHeight = cameraImage.description.height
            CameraWidth = cameraImage.description.width

            lock.acquire()
            PH_Camera = np.frombuffer(cameraImage.pixelData, dtype=np.uint8)
            PH_Camera.shape = CameraHeight, CameraWidth, 3
            lock.release()
            # cv2.imshow('PH_Camera', PH_Camera)


    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def txCamera(Image):

    status = 0
    ic = None


    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Camera:default -p 9000")
        datos = jderobot.CameraPrx.checkedCast(base)
        # print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        # global PH_Camera
        # PH_Camera = datos.startCameraStreaming()

        while True:
            image2send = Image
            datos.setCMDVelData(image2send)


    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

def velVector(position,waypoint):

    vector = []
    for i in range(len(position)):
        vector.append(waypoint[i]-position[i])

    module = math.sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2])

    for i in range(len(vector)):
        vector[i] = vector[i]/module

    tuple(vector)
    return vector

def nextWaypointCMDVel(position, trajectory):

    waypoint = trajectory[0]
    vector = velVector(position, waypoint)
    print "Waypoint: %s" %waypoint
    dist = distance(waypoint, position)
    print "distance: %f" %dist

    if (dist <= ReachedDist):
        trajectory = trajectory[1:len(trajectory)] #pop
        print "Waypoint reached"

    return vector,trajectory

def nextWaypointPose3D(position, trajectory):

    waypoint = trajectory[0]
    #vector = velVector(position, waypoint)
    print "Waypoint: %s" %waypoint
    dist = distance(waypoint, position)
    print "distance: %f" %dist

    if (dist <= ReachedDist):
        trajectory = trajectory[1:len(trajectory)] #pop
        print "Waypoint reached"

    return trajectory

def distance(pos1,pos2):

    dist = []
    for i in range(len(pos1)):
        dist.append(pos1[i] - pos2[i])

    module = math.sqrt(dist[0]*dist[0] + dist[1]*dist[1] + dist[2]*dist[2])
    module = math.fabs(module)

    return module

def spiralTrajectory(startWaypoint, scanDistance, spinsNumber):

    trajectory = []
    trajectory.append(startWaypoint)

    x = StartWaypoint[0]
    y = StartWaypoint[1]
    z = MissionHeight


    for i in range(spinsNumber):

        x = x + scanDistance*(2*i+1)
        trajectory.append([x,y,z])
        y = y + scanDistance*(2*i+1)
        trajectory.append([x,y,z])

        x = x - scanDistance*(2*i+2)
        trajectory.append([x,y,z])
        y = y - scanDistance*(2*i+2)
        trajectory.append([x,y,z])

    return trajectory

def detection(image, hmin, hmax, smin, smax, vmin, vmax):
    # ------- Thresold Image -------#

    rawImage = image

    ksize = (5, 5)
    sigma = 9
    gaussImage = cv2.GaussianBlur(rawImage, ksize, sigma)

    hsvImage = cv2.cvtColor(gaussImage, cv2.COLOR_RGB2HSV)

    matrixMin = np.array([hmin, smin, vmin])
    matrixMax = np.array([hmax, smax, vmax])

    filteredImage = cv2.inRange(hsvImage, matrixMin, matrixMax)

    # ------- Detect Object -------#

    center = []
    area = 0
    stillLost = True


    detImage = rawImage

    contour, hierarchy = cv2.findContours(filteredImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contour) > 0:
        contourdx = -1
        cv2.drawContours(detImage, contour, contourdx, (255, 255, 0))

    for i in range(len(contour)):
        contarray = contour[i]

        epsilon = 5
        closed = True
        approxCurve = cv2.approxPolyDP(contarray, epsilon, closed)

        rectangle = cv2.boundingRect(approxCurve)
        rectX, rectY, rectW, rectH = rectangle

        if rectW < maxTargetLado and rectW > minTargetLado and rectH < maxTargetLado and rectH > minTargetLado:
            myRectangle = rectangle
        else:
            myRectangle = 0, 0, 0, 0

        myRectX, myRectY, myRectW, myRectH = myRectangle
        myPoint1 = myRectX, myRectY
        myPoint2 = myRectX + myRectW, myRectY + myRectH
        center = myRectX + (myRectW / 2), myRectY + (myRectH / 2)
        area = myRectW * myRectH

        # cv2.rectangle(detImage,myPoint1,myPoint2,(255,0,0),2)
        cv2.circle(detImage, center, 1, (255, 0, 0), 2)

        minTargetArea = minTargetLado * minTargetLado

        if (area > minTargetArea):
            # print center
            # print area
            stillLost = False


    return center, area, stillLost

if __name__ == '__main__':

    PH_Pose3D = Pose3DI(0,0,0,0,0,0,0,0)

    PoseTheading = threading.Thread(target=rxPose3D, args=(PH_Pose3D,), name='ClientPHPose_Theading')
    PoseTheading.daemon = True
    PoseTheading.start()

    WP_Pose3D = Pose3DI(0,0,0,0,0,0,0,0)

    PoseTheading = threading.Thread(target=txPose3DWP2Server, args=(WP_Pose3D,), name='ClientWPPose_Theading')
    PoseTheading.daemon = True
    PoseTheading.start()


    PH_CMDVel = CMDVelI(0,0,0,0,0,0)

    CMDVelTheading = threading.Thread(target=txCMDVel2Server, args=(PH_CMDVel,), name='ClientCMDVel_Theading')
    CMDVelTheading.daemon = True
    CMDVelTheading.start()


    # PH_Extra = ExtraI()
    #
    # CMDVelTheading = threading.Thread(target=txExtra2Server, args=(PH_Extra,), name='ClientExtra_Theading')
    # CMDVelTheading.daemon = True
    # CMDVelTheading.start()


    # PH_Camera = None #np.ones((1080, 1080, 3), np.uint8)

    CameraTheading = threading.Thread(target=rxCamera, args=(), name='ClientCamera_Theading')
    CameraTheading.daemon = True
    CameraTheading.start()


    DefTrajectory = spiralTrajectory(StartWaypoint, ScanDistance, SpinsNumber)
    trajectory = DefTrajectory
    print 'Trajectory:'
    print trajectory

    time.sleep(2) #due to that all Ice channel need to be initialized

    while True:

        time.sleep(1)#0.05) #20Hz
        position = PH_Pose3D.getPose3DData()
        xyz = [position.x, position.y, position.z]
        landDecision = False;

        lock.acquire()
        Image2Analyze = PH_Camera
        ImageShape = PH_Camera.shape
        lock.release()

        GreenCenter, GreenArea, GreenStillLost = detection(Image2Analyze, hminG, hmaxG, sminG, smaxG, vminG, vmaxG)
        OrangeCenter, OrangeArea, OrangeStillLost = detection(Image2Analyze, hminO, hmaxO, sminO, smaxO, vminO, vmaxO)


        #FILTER NEEDED!!!!

        targetFound = not(GreenStillLost or OrangeStillLost)
        totalArea = GreenArea + OrangeArea

        # print GreenStillLost
        # print OrangeStillLost
        # print stillLost


        if targetFound:

            print 'Target Found'
            print str(GreenCenter)
            print str(GreenArea)

            imageCentre = (int(CameraWidth) / 2, int(CameraHeight) / 2)

            cv2.line(Image2Analyze, (0, imageCentre[0]), (2 * imageCentre[1], imageCentre[1]), (255, 255, 255), )
            cv2.line(Image2Analyze, (imageCentre[0], 0), (imageCentre[1], 2 * imageCentre[1]), (255, 255, 255), )

            xPid = PID(HorP, HorD, HorI)
            xError = imageCentre[0] - GreenCenter[0]
            xPid.setError(xError)
            controlX = xPid.calcControl()

            yPid = PID(HorP, HorD, HorI)
            yError = imageCentre[1] - GreenCenter[1]
            yPid.setError(xError)
            controlY = yPid.calcControl()

            zPid = PID(VerP, VerD, VerI)
            zError = MissionHeight - xyz[2]  # z
            zPid.setError(xError)
            controlZ = zPid.calcControl()


            #Landing decision make
            landDecision = True
            data = jderobot.CMDVelData()

            if landDecision:
                data.linearZ = -1 #Landing Velocity
            else:
                data.linearZ = 0

            PH_CMDVel.setCMDVelData(data)

        else:

            print 'Searching for target'

            # command, updatedTrajectory = nextWaypointCMDVel(xyz, trajectory)
            updatedTrajectory = nextWaypointPose3D(xyz, trajectory)
            trajectory = updatedTrajectory

            #restart trajectory when finish
            if (len(trajectory) == 0):
                trajectory = DefTrajectory
                print "Trajectory restarted:"
                print trajectory

            #send waypoint to server
            wayPoint = jderobot.Pose3DData()
            wayPoint.x = 10 * random.random()  # trajectory[0][0]
            wayPoint.y = 10 * random.random()  # trajectory[0][1]
            wayPoint.z = 10 * random.random()  # trajectory[0][2]
            WP_Pose3D.setPose3DData(wayPoint)
            # print wayPoint


            #Not to land
            data = jderobot.CMDVelData()
            data.linearZ = 0
            PH_CMDVel.setCMDVelData(data)






