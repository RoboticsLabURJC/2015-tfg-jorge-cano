__author__ = 'AeroCano'

import jderobot
import sys, traceback, Ice

class Pose3DI(jderobot.Pose3D):

    def __init__(self,_x,_y,_z,_h,_q0,_q1,_q2,_q3):

        self.x = _x
        self.y = _y
        self.z = _z
        self.h = _h
        self.q0 = _q0
        self.q1 = _q1
        self.q2 = _q2
        self.q3 = _q3

    def setPose3DData(self,_x,_y,_z,_h,_q0,_q1,_q2,_q3):

        self.x = _x
        self.y = _y
        self.z = _z
        self.h = _h
        self.q0 = _q0
        self.q1 = _q1
        self.q2 = _q2
        self.q3 = _q3


    def getPose3DData(self, current=None):

        data = jderobot.Pose3DData()
        data.x = self.x
        data.y = self.y
        data.z = self.z
        data.h = self.h
        data.q0 = self.q0
        data.q1 = self.q1
        data.q2 = self.q2
        data.q3 = self.q3

        return data

    def printPose3DData(self,pose):

        print self.x
        print self.y
        print self.z
        print self.h
        print self.q0
        print self.q1
        print self.q2
        print self.q3

    def TxPose3D(self):
        status = 0
        ic = None
        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("PHPose3DAdapter", "default -p 9998")
            object = Pose3DI(self.x, self.y, self.z, self.h, self.q0, self.q1, self.q2, self.q3)
            print object.getPose3DData()
            adapter.add(object, ic.stringToIdentity("PHPose3D"))
            adapter.activate()
            ic.waitForShutdown()
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

        sys.exit(status)
