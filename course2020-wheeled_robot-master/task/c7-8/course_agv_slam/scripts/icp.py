#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import time
from scipy.spatial import cKDTree as KDTree
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
        self.max_iter = rospy.get_param('/icp/max_iter',30)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',5)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance',0.0001)
        # if is the first scan, set as the map/target
        self.isFirstScan = True
        # src point cloud matrix
        self.src_pc = []
        # target point cloud matrix
        self.tar_pc = []
        self.map_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
    def laserCallback(self,msg):
        if(msg.ranges[0]>999 or msg.ranges[3]>999  or msg.ranges[3]<-999  or msg.ranges[0]<-999):
            return 
        #print("msg")
        #print(msg)
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(msg)
            self.isFirstScan = False
            self.laser_count = 0
            return
        
        # process every 5 laser scan for laser fps too high
        self.laser_count += 1
        if self.laser_count <= 5:
            return


        self.laser_count = 0
        time_0 = rospy.Time.now()
        self.src_pc = self.laserToNumpy(msg)
        print('input cnt: ',self.src_pc.shape[1])

        # init some variables
        transform_acc = np.identity(3)
        m = self.src_pc.shape[1]
        #mysrc = np.ones((m+1,self.src_pc.shape[0]))
        #mydst = np.ones((m+1,self.tar_pc.shape[0]))
        mysrc= np.copy(self.src_pc)
        mydst= np.copy(self.tar_pc)
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
        transform_acc=T
 
    # make points homogeneous, copy them to maintain the originals
        print("total_iter: ",iter_cnt)
        self.tar_pc = self.laserToNumpy(msg)
        self.publishResult(transform_acc)
        time_1 = rospy.Time.now()
        print("time_cost: ",time_1-time_0)
        pass

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
        if Hin is None:
            return H
        else:
            return np.dot(Hin,H)

    
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


    def publishResult(self,T):
        # print("t: ",T)
        delta_yaw = math.atan2(T[1,0],T[0,0])
        print("sensor-delta-xyt: ",T[0,2],T[1,2],delta_yaw)
        s = self.sensor_sta
        #self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        #self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        #self.sensor_sta[2] = s[2] + delta_yaw
        self.sensor_sta[0] = T[0,2]
        self.sensor_sta[1] = T[1,2]
        self.sensor_sta[2] = delta_yaw
        print("sensor-global: ",self.sensor_sta)

        # tf
        s = self.sensor_sta
        q = tf.transformations.quaternion_from_euler(0,0,self.sensor_sta[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_odom","world_base")
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        print("tfsend")
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        pass
    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

def main():
    rospy.init_node('icp_node')
    icp = ICP()
    rospy.spin()

if __name__ == '__main__':
    main()