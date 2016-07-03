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

MissionHeight = 5      #metres
StartWayPointLatLonHei = {}
StartWayPointLatLonHei['lat'] = math.radians(40.191240)
StartWayPointLatLonHei['lon'] = math.radians(-3.855655)
StartWayPointLatLonHei['hei'] = MissionHeight
ScanDistance = 3        #metres
SpinsNumber = 2
ReachedDist = 1         #metres
LandingPrecision = 2    #metres

# --- Camera ---#

CameraFOW = 120  # Field of View in degrees

# --- Target ---#

minTargetLado = 20
maxTargetLado = 1000

# Green
hminG = 50
hmaxG = 80
vminG = 80
vmaxG = 255
sminG = 110
smaxG = 255

# Orange
hminO = 10
hmaxO = 35
vminO = 80
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

    module = 0
    for i in range(len(pos1)):
        module = module + dist[i]*dist[i]

    module = math.sqrt(module)
    module = math.fabs(module)

    return module

def spiralTrajectory(startWP, scanDistance, spinsNumber):

    trajectory = []

    x = startWP['x']
    y = startWP['y']
    z = startWP['z']

    firstWP = [x, y, z]
    trajectory.append(firstWP)

    scanDistanceLatLon = scanDistance

    for i in range(spinsNumber):

        x = x + scanDistanceLatLon*(2*i+1)
        trajectory.append([x,y,z])
        y = y + scanDistanceLatLon*(2*i+1)
        trajectory.append([x,y,z])

        x = x - scanDistanceLatLon*(2*i+2)
        trajectory.append([x,y,z])
        y = y - scanDistanceLatLon*(2*i+2)
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
    center = (0,0)
    colorFound = False


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

        if (area > minTargetArea)and(center != (0,0)):
            colorFound = True
        else:
            colorFound = False

    return center, area, colorFound

def global2cartesian(poseLatLonHei):

    wgs84_radius = 6378137 #meters
    wgs84_flattening = 1 - 1 / 298.257223563
    eartPerim = wgs84_radius * 2 * math.pi

    earthRadiusLon = wgs84_radius * math.cos(poseLatLonHei['lat'])/wgs84_flattening
    eartPerimLon = earthRadiusLon * 2 * math.pi

    poseXYZ = {}
    poseXYZ['x'] = poseLatLonHei['lon'] * eartPerimLon / (2*math.pi)
    poseXYZ['y'] = poseLatLonHei['lat'] * eartPerim / (2*math.pi)
    poseXYZ['z'] = poseLatLonHei['hei']
    return poseXYZ

def pixel2metres(pixelXY, pixelNum, height, fov):

    height = MissionHeight #only for testing
    vehicleAngle = fov/2
    groundAngle = (math.pi/2) - vehicleAngle
    hipoten = height / math.sin(groundAngle)
    groundSide = math.sqrt((hipoten*hipoten) - (height*height))

    metersXpixel = 2*groundSide / pixelNum


    metresXY = [metersXpixel*pixelXY[0] , metersXpixel*pixelXY[1]]

    return metresXY

def frameChange2D(vector, angle):

    rotatedX = vector[0]*math.cos(angle) + vector[1]*math.sin(angle)
    rotatedY = vector[1]*math.cos(angle) - vector[0]*math.sin(angle)

    rotatedVector = (rotatedX,rotatedY)

    return rotatedVector

def yawFromQuaternion(Pose3D):

    q0 = Pose3D.q0
    q1 = Pose3D.q1
    q2 = Pose3D.q2
    q3 = Pose3D.q3

    yaw = math.atan2(2*(q0*q3 + q1*q2) , 1 - 2*((q2*q2)+(q3*q3)))
    return yaw

class Buffer:

    def __init__(self, size):
        self.size = size
        self.data = [False for i in xrange(size)]

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data

    def decicion(self):

        count = 0
        threshold = len(self.data)/2 #50%

        for i in range(self.size):
            if self.data[i]:
                count = count + 1

        decision = count >= threshold

        return decision

def qMultiply (q1,q2):

    q1 = qNormal(q1)
    q2 = qNormal(q2)

    # quaternion1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    #quaternion2
    w2 = q2[0]
    x2 = q2[1]
    y2 = q2[2]
    z2 = q2[3]

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2

    q = [w,x,y,z]

    q = qNormal(q)
    return q

def qNormal(q1):

    qmodule = math.sqrt(q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2] + q1[3]*q1[3])
    q = [0,0,0,0]

    if (qmodule == 0):
        qmodule = 0.000000000001

    q[0] = q1[0] / qmodule
    q[1] = q1[1] / qmodule
    q[2] = q1[2] / qmodule
    q[3] = q1[3] / qmodule

    return q

def qConjugate(q1):

    q1 = qNormal(q1)
    q = [0,0,0,0]
    q[0] = q1[0]
    q[1] = -q1[1]
    q[2] = -q1[2]
    q[3] = -q1[3]

    q = qNormal(q)
    return q

def qInverse(q1):

    q1 = qNormal(q1)
    qconjugate = qConjugate(q1)
    qmodule = math.sqrt(q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3])

    if (qmodule == 0):
        qmodule = 0.000000000001

    q = [0,0,0,0]
    q[0] = qconjugate[0] / qmodule
    q[1] = qconjugate[1] / qmodule
    q[2] = qconjugate[2] / qmodule
    q[3] = qconjugate[3] / qmodule

    q = qNormal(q)
    return q

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

    StartWayPointXYZ = global2cartesian(StartWayPointLatLonHei)
    DefTrajectory = spiralTrajectory(StartWayPointXYZ, ScanDistance, SpinsNumber)
    trajectory = DefTrajectory
    print 'Trajectory:'
    print trajectory

    targetBuffer = Buffer(10)
    targetFound = False
    landDecision = False



    time.sleep(2) #Ice channels need to be initialized

    while True:

        time.sleep(1)#0.05) #20Hz
        vehiclePose = PH_Pose3D.getPose3DData()
        vehicleXYZ = [vehiclePose.x, vehiclePose.y, vehiclePose.z]
        vehicleYaw = yawFromQuaternion(vehiclePose)

        lock.acquire()
        Image2Analyze = PH_Camera
        ImageShape = PH_Camera.shape
        lock.release()

        GreenCenter, GreenArea, GreenFound = detection(Image2Analyze, hminG, hmaxG, sminG, smaxG, vminG, vmaxG)
        OrangeCenter, OrangeArea, OrangeFound = detection(Image2Analyze, hminO, hmaxO, sminO, smaxO, vminO, vmaxO)
        targetArea = GreenArea + OrangeArea

        # print GreenCenter
        # print OrangeCenter
        # print GreenArea
        # print OrangeArea

        twoAreas = GreenFound and OrangeFound
        nearAreas = distance(GreenCenter,OrangeCenter) <= math.sqrt(targetArea)

        if twoAreas and nearAreas:
            targetCentrePixels = (((GreenCenter[0] + OrangeCenter[0]) / 2), ((GreenCenter[1] + OrangeCenter[1]) / 2))
            targetBuffer.append(True)
        else:
            targetBuffer.append(False)

        targetBufferData = targetBuffer.get()

        targetFound = targetBuffer.decicion()
        print targetBufferData

        if targetFound:

            print 'Target Found'

            CameraFOWrad = math.radians(CameraFOW)
            cameraCentrePixels = (ImageShape[0]/2, ImageShape[1]/2)
            cameraCentreMetres = pixel2metres(cameraCentrePixels, ImageShape[0], vehicleXYZ[2], CameraFOWrad)
            targetCentreMetres = pixel2metres(targetCentrePixels, ImageShape[0], vehicleXYZ[2], CameraFOWrad)
            landingError = distance(targetCentreMetres, cameraCentreMetres)

            ### Landing decision make  ###
            print 'Landing error: %f' %landingError

            # Loiter if target is in the landing area #
            if (landingError < LandingPrecision) and (landingError != 0):
                print 'Landing decision = TRUE'
                targetCentreMetres = (0,0)
                landDecision = True

            data = jderobot.CMDVelData()

            if landDecision:
                data.linearZ = -1  # Landing Velocity
            else:
                data.linearZ = 0

            PH_CMDVel.setCMDVelData(data)

            ### send waypoint to server ###
            targetCentreMetresDiff = frameChange2D(targetCentreMetres, vehicleYaw)
            wayPoint = jderobot.Pose3DData()
            wayPoint.x = vehicleXYZ[0] + targetCentreMetres[0]
            wayPoint.y = vehicleXYZ[1] + targetCentreMetres[1]
            wayPoint.z = MissionHeight
            WP_Pose3D.setPose3DData(wayPoint)
            # print wayPoint



        else:

            print 'Searching for target'

            # command, updatedTrajectory = nextWaypointCMDVel(xyz, trajectory)
            updatedTrajectory = nextWaypointPose3D(vehicleXYZ, trajectory)
            trajectory = updatedTrajectory

            #restart trajectory when finish
            if (len(trajectory) == 0):
                trajectory = DefTrajectory
                print "Trajectory restarted:"
                print trajectory

            #send waypoint to server
            wayPoint = jderobot.Pose3DData()
            wayPoint.x = trajectory[0][0]
            wayPoint.y = trajectory[0][1]
            wayPoint.z = trajectory[0][2]
            WP_Pose3D.setPose3DData(wayPoint)
            # print wayPoint


            # #Not to land
            # data = jderobot.CMDVelData()
            # data.linearZ = 0
            # PH_CMDVel.setCMDVelData(data)






