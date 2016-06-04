__author__ = 'AeroCano'

import jderobot, time, Ice

class CMDVelI(jderobot.CMDVel):


    #def setjorge(self,data,c):

     #   Ice.setCMDVelData()

        # self.linearX = data.linearX
        # self.linearY = data.linearY
        # self.linearZ = data.linearZ
        # self.angularX = data.angularX
        # self.angularY = data.angularY
        # self.angularZ = data.angularZ

    def getCMDVelData(self, current=None):

        time.sleep(0.05)  # 50 ms rate to rx CMDVel
        data = jderobot.CMDVelData()
        data.linearX = self.linearX
        data.linearY = self.linearY
        data.linearZ = self.linearZ
        data.angularX = self.angularX
        data.angularY = self.angularY
        data.angularZ = self.angularZ

        return data
