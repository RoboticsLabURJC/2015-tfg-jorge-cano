__author__ = 'JorgeCano'

import sys, traceback, Ice
import jderobot, time
import random
import math
import numpy as np

from Pose3D import Pose3DI
from CMDVel import CMDVelI

import threading


##### Globals #####

MissionHeight = 2    #metres
StartWaypoint = [1,1,MissionHeight]
ScanDistance = 5        #metres
SpinsNumber = 3
ReachedDist = 0.5    #metres

###################

def rxPose3D(Pose3D):

    status = 0
    ic = None
    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("Pose3D:default -p 9000")
        datos = jderobot.Pose3DPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(1)
            data = datos.getPose3DData()

            PHx = data.x
            PHy = data.y
            PHz = data.z
            PHh = data.h
            PHq0 = data.q0
            PHq1 = data.q1
            PHq2 = data.q2
            PHq3 = data.q3
            Pose3D.setPose3DData(PHx,PHy,PHz,PHh,PHq0,PHq1,PHq2,PHq3)
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

def txCMDVel2Server(CMDVel):

    status = 0
    ic = None

    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("CMDVel:default -p 9000")
        datos = jderobot.CMDVelPrx.checkedCast(base)
        #print datos
        if not datos:
            raise RuntimeError("Invalid proxy")

        while True:
            time.sleep(1)
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

def velVector(position,waypoint):

    vector = []
    for i in range(len(position)):
        vector.append(waypoint[i]-position[i])

    module = math.sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2])

    for i in range(len(vector)):
        vector[i] = vector[i]/module

    tuple(vector)
    return vector

def nextWaypoint(position, trajectory):

    waypoint = trajectory[0]
    vector = velVector(position, waypoint)
    print "target: %s" %waypoint
    dist = distance(waypoint, position)
    print "distance: %f" %dist

    if (dist <= ReachedDist):
        trajectory = trajectory[1:len(trajectory)] #pop
        print "Waypoint reached"

    return vector,trajectory

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

def targetLost():

    stillLost = True
    return stillLost

if __name__ == '__main__':

    PH_Pose3D = Pose3DI(0,0,0,0,0,0,0,0)

    PoseTheading = threading.Thread(target=rxPose3D, args=(PH_Pose3D,), name='ClientPose_Theading')
    PoseTheading.daemon = True
    PoseTheading.start()


    PH_CMDVel = CMDVelI(0,0,0,0,0,0)

    CMDVelTheading = threading.Thread(target=txCMDVel2Server, args=(PH_CMDVel,), name='ClientCMDVel_Theading')
    CMDVelTheading.daemon = True
    CMDVelTheading.start()

    spiralTrajectory(StartWaypoint, ScanDistance, SpinsNumber)

    DefTrajectory = spiralTrajectory(StartWaypoint, ScanDistance, SpinsNumber)
    trajectory = DefTrajectory
    print trajectory

    while targetLost:
        time.sleep(1)

        position = PH_Pose3D.getPose3DData()
        print position
        xyz= [position.x,position.y,position.z]
        command,updatedTrajectory = nextWaypoint(xyz, trajectory)

        trajectory = updatedTrajectory

        if (len(trajectory)== 0):
            trajectory = DefTrajectory
            print "trajectory restarted"
            print trajectory

        print command

        data = jderobot.CMDVelData()
        data.linearX = command[0]
        data.linearY = command[1]
        data.linearZ = command[2]

        PH_CMDVel.setCMDVelData(data)

        #print PH_CMDVel.getCMDVelData()


    while True:
        time.sleep(1)
        print "OLE"



