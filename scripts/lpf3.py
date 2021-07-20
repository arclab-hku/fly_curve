import numpy as np

"""
# Third-order low pass filter
# \frac{X_f(s)}{X_m(s)} = \frac{\omega_1\omega_2^2}{(s^2+\omega_1)(s^2+2\zeta\omega_2s+\omega_2^2)}
"""
class LowPassFilter3:
    def __init__(self, INTV, ZETA=0.8, OMEGA1=100, OMEGA2=100):
        self.h = INTV        # sample interval
        self.__zeta = ZETA
        self.__omega1 = OMEGA1
        self.__omega2 = OMEGA2

        self.a31 = -self.__omega1 * self.__omega2 * self.__omega2
        self.a32 = -(self.__omega2 * self.__omega2 + 2 * self.__zeta * self.__omega2 * self.__omega1)
        self.a33 = -(self.__omega1 + 2 * self.__zeta * self.__omega2)

        self.x1 = np.zeros([3,1])       # x1 = \hat x
        self.x2 = np.zeros([3,1])       # x2 = \dot \hat x
        self.x3 = np.zeros([3,1])       # x3 = \ddot \hat x
        self.xm = np.zeros([3,1])       # measurement signal

    # a third-order lpf function
    def filter(self, Xm, INTV):         # Xm, new measurement, 3*1 vector; INTV, sample interval
        self.h = INTV        # sample interval

        # input
        self.xm = Xm
        print("input:",self.xm)

        # system
        self.x3 = Runge_Kutta4(self.dotx3, self.x3, self.h)
        self.x2 = Runge_Kutta4(self.dotx2, self.x2, self.h)
        self.x1 = Runge_Kutta4(self.dotx1, self.x1, self.h)

        print("filter:",self.x1)
        return self.x1

    # state transfer functions of the filter
    def dotx1 (self, y):        
        return self.x2

    def dotx2 (self, y):
        return self.x3

    def dotx3 (self, y):
        return self.a31 * self.x1 + self.a32 * self.x2 + self.a33 * y - self.a31 * self.xm

# the Fourth-Order Runge-Kutta for simulating the filter system
def Runge_Kutta4 (func, yt, h):     # t: current time; yt: current value; h: time interval
    k1 = func(yt)
    k2 = func(yt+(h/2.0)*k1)
    k3 = func(yt+(h/2.0)*k2)
    k4 = func(yt+h*k3)

    return yt + h/6.0*(k1+2*k2+2*k3+k4)