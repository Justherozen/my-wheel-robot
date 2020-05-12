import math
import numpy as np


#DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

# Covariance for EKF simulation
#Q = np.diag([
#    0.2,  # variance of location on x-axis
#    0.2,  # variance of location on y-axis
#    np.deg2rad(3.0)  # variance of yaw angle
#]) ** 2  # predict state covariance
#R = np.diag([0.2, 0.2,np.deg2rad(3.0)]) ** 2  # Observation x,y position covariance

Q = np.diag([1.0, 1.0])**2  # Observation x,y position covariance
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2

#DT = 0.1  # time tick [s]

class EKF():
    def __init__(self):
        self.DT = 0.1
        pass
    def odom_model(self,x,u):
        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[self.DT * math.cos(x[2, 0]), 0],
                    [self.DT * math.sin(x[2, 0]), 0],
                    [0.0, self.DT],
                    [1.0, 0.0]])

        x = F.dot(x) + B.dot(u)
        return x



    def observation_model(self,x):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        z = H.dot(x)
        return z   

    def jacob_f(self,x, u):

        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
            [0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF


    def jacob_h(self):
        # Jacobian of Observation Model
        jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0] ])
        return jH



    def estimate(self,xEst, PEst, z, u):
        #  Predict
        xPred = self.odom_model(xEst, u)    
        jF = self.jacob_f(xPred, u)      #jF 4*4
        PPred = jF.dot(PEst).dot(jF.T) + R   #4*4
        #  Update
        jH = self.jacob_h()                 #2*4 
        zPred = self.observation_model(xPred)   #zPred 2*1
        y = z.T - zPred                #Y 2*1
        S = jH.dot(PPred).dot(jH.T) + Q   #S 2*2
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))# K 4*2
        xEst = xPred + K.dot(y)                   #K*y   4*1   X  4*1  
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)  #PEst  4*4

        return xEst, PEst
####





