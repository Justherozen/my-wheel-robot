#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import time
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class ICP:
    def __init__(self): 
        self.laser_count = 0
        # robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        # sensor states = robot_x_y_theta
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.H=None
        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter',50)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',5)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance',0.001)
        # if is the first scan, set as the map/target
        self.isFirstScan = True
        # src point cloud matrix
        self.src_pc = []
        # target point cloud matrix
        self.tar_pc = []
        #self.map_broadcaster = tf.TransformBroadcaster()
        #self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
         #                   rospy.Time.now(),"world_base","map")
        #self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        #self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=1)
        #self.odom_broadcaster = tf.TransformBroadcaster()
    def process(self,mysrc,mydst):#mydst to mysrc
        # init some variables
        transform_acc = np.identity(3)
        #m = self.src_pc.shape[1]
        #mysrc = np.ones((m+1,self.src_pc.shape[0]))
        #mydst = np.ones((m+1,self.tar_pc.shape[0]))
        #print("src &dst")
        #print(mysrc)
        #print(mydst)
        prev_error = 0
        iter_cnt=0
        for _ in range(self.max_iter):
            indexes,error= self.findNearest(mydst, mysrc)
            #print(distances)
            #print(indices)
            R,T= self.getTransform(mydst[:, indexes],mysrc)
            mysrc = (np.dot(R,mysrc)) + T[:, np.newaxis]
            self.H = self.update_homogeneous_matrix(self.H, R, T)
            #mean_error = np.mean(distances)
            dError = abs(prev_error - error)
            if dError < self.tolerance:
                break
            prev_error = error                
            iter_cnt += 1
            pass
        T=self.H
        self.H=None
        transform_acc=T
        return transform_acc
    # make points homogeneous, copy them to maintain the originals
        #print("total_iter: ",iter_cnt)
        #self.tar_pc = self.laserToNumpy(msg)
        #self.publishResult(transform_acc)
        #time_1 = rospy.Time.now()
        #print("time_cost: ",time_1-time_0)
        #self.laser_count =0
        #pass

    def findNearest(self,previous_points, current_points):
        
        # calc the sum of residual errors
        delta_points = previous_points - current_points
        d = np.linalg.norm(delta_points, axis=0)
        error = sum(d)

        # calc index with nearest neighbor assosiation
        d = np.linalg.norm(np.repeat(current_points, previous_points.shape[1], axis=1)
                        - np.tile(previous_points, (1, current_points.shape[1])), axis=0)
        indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)

        return indexes, error
    
    def update_homogeneous_matrix(self,Hin, R, T):
        
        H = np.zeros((3, 3))

        H[0, 0] = R[0, 0]
        H[1, 0] = R[1, 0]
        H[0, 1] = R[0, 1]
        H[1, 1] = R[1, 1]
        H[2, 2] = 1.0
        H[2, 0] = 0
        H[2, 1] = 0
        H[0, 2] = T[0]
        H[1, 2] = T[1]
        return H
    
    

    
    def getTransform(self,src,tar):
        T = np.identity(3)
        pm = np.mean(src, axis=1)
        cm = np.mean(tar, axis=1)
        p_shift = src- pm[:, np.newaxis]
        c_shift = tar - cm[:, np.newaxis]        
        W = np.dot(c_shift,p_shift.T)

        #print(c_shift)
        #print(p_shift)
        #print("w is")
        #print(W)
        try:
            u, s, vh = np.linalg.svd(W)
        except Exception as e:
            print(e)
            print(src)
            print(tar)

        R = (np.dot(u,vh)).T
        T = pm - (np.dot(R,cm))
        #print("this is T2")
        #print(T)
        return R, T


    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

