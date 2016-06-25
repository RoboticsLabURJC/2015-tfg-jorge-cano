__author__ = 'JCano'

class PID():

    def __init__(self, my_Kp, my_Ki, my_Kd):
        self.Kp = my_Kp
        self.Ki = my_Ki
        self.Kd = my_Kd
        self.error = 0
        #self.control = my_control

    def setKp(self, value):
        self.Kp = value

    def setKi(self, value):
        self.Ki = value

    def setKd(self, value):
        self.Kd = value

    def getKp(self):
        return self.Kp

    def getKi(self):
        return self.Ki

    def getKd(self):
        return self.Kd

    def setError(self, value):
        self.diffError = value - self.error
        self.intError = value + self.error
        self.error = value

    def getError(self):
        return self.error

    def calcControl(self):
        self.control = (self.Kp * self.error) + \
                       (self.Kd * self.diffError) + \
                       (self.Ki * self.intError)  # not tested
        return self.control