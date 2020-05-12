#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan,PointCloud
from nav_msgs.msg import Odometry,Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped,Point32
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Twist, PointStamped,PoseStamped
from sensor_msgs.msg import ChannelFloat32
import numpy as np
import math
import time
from scipy.spatial import cKDTree as KDTree
import sys
import copy
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class ICP:
    def __init__(self): 
        self.tmpposes=[]
        self.icppoints=[]
        self.icppoints2=[]
        self.realpath=Path()
        self.realpath.poses=[]
        self.icpcloud=PointCloud()
        self.icpcloud2=PointCloud()
        self.laser_count = 0
        self.obstacle_r = 10
        self.obstacle = []
        self.target_laser=LaserScan()
        # robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        # sensor states = robot_x_y_theta
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.H=None
        self.H2=None
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
        self.ver_pc = []
        # target point cloud matrix
        self.tar_pc = []
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        self.map_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.laser_sub = rospy.Subscriber('/map',OccupancyGrid,self.updateMap)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom_broadcaster_2 = tf.TransformBroadcaster()
        self.tf = tf.TransformListener()
        self.xEst = np.zeros((3,1))
        self.txEst = np.zeros((3,1))
        self.xdom = np.zeros((3,1))
        self.realpath_pub = rospy.Publisher('realpath',Path,queue_size=1)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=1)
        self.icpp_pub = rospy.Publisher('ekff',PointCloud,queue_size=1)
        self.icpp_pub2 = rospy.Publisher('icpp',PointCloud,queue_size=1)
        self.indexi=0

    def laserCallback(self,msg):
        if(msg.ranges[0]>999 or msg.ranges[3]>999  or msg.ranges[3]<-999  or msg.ranges[0]<-999):
            return 
        ##print("msg")
        ##print(msg)
        #print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(msg)
            self.target_laser=self.laserEstimation(msg,self.xEst)
            self.ver_pc=self.laserToNumpy(self.target_laser)
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
        #print('input cnt: ',self.src_pc.shape[1])
        # init some variables
        transform_acc = np.identity(3)
        ekf_acc = np.identity(3)
        m = self.src_pc.shape[1]
        #mysrc = np.ones((m+1,self.src_pc.shape[0]))
        #mydst = np.ones((m+1,self.tar_pc.shape[0]))
        mysrc_1= np.copy(self.src_pc)
        mysrc_2= np.copy(self.src_pc)
        myver= np.copy(self.ver_pc)
        mydst= np.copy(self.tar_pc)
        ##print("src &dst")
        ##print(mysrc)
        ##print(mydst)
        prev_error = 0
        iter_cnt=0


        for _ in range(self.max_iter):
            indexes,error= self.findNearest(myver, mysrc_1)
            ##print(distances)
            ##print(indices)
            R,T= self.getTransform(myver[:, indexes],mysrc_1)
            mysrc_1 = (np.dot(R,mysrc_1)) + T[:, np.newaxis]
            self.H2 = self.update_homogeneous_matrix(self.H2, R, T)
            #mean_error = np.mean(distances)
            dError = abs(prev_error - error)
            if dError < self.tolerance:
                break
            prev_error = error                
            iter_cnt += 1
            pass
        #print("total_iter: ",iter_cnt)



        prev_error = 0
        iter_cnt=0

        for _ in range(self.max_iter):
            indexes,error= self.findNearest(mydst, mysrc_2)
            ##print(distances)
            ##print(indices)
            R,T= self.getTransform(mydst[:, indexes],mysrc_2)
            mysrc_2 = (np.dot(R,mysrc_2)) + T[:, np.newaxis]
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
        ekf_acc=self.H2
        self.xdom[0] =self.H[0,2]
        self.xdom[1]=self.H[1,2]
        self.xdom[2]=math.atan2(self.H[1,0],self.H[0,0])
        ekf_yaw = math.atan2(transform_acc[1,0],transform_acc[0,0])

        self.xEst[0]=ekf_acc[0,2]
        self.xEst[1]=ekf_acc[1,2]
        self.xEst[2]=math.atan2(self.H2[1,0],self.H2[0,0])
        #print("H2")
        #print(self.H2)
        #print(self.H)
        
        try:
            #print("getmytf1")
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
            #print("getmytf2")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.txEst[0] = self.trans[0]#+np.random.uniform(-1,1,1)*0.03
        self.txEst[1] = self.trans[1]#+np.random.uniform(-1,1,1)*0.03
        self.txEst[2] = yaw#+np.random.uniform(-1,1,1)*0.02 
        self.realpath.header.seq = 0
        self.realpath.header.stamp = rospy.Time(0)
        self.realpath.header.frame_id = 'map'
        pose = PoseStamped()
        pose.header.seq = self.indexi
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.x = copy.deepcopy(self.txEst[0,0])
        pose.pose.position.y = copy.deepcopy(self.txEst[1,0])
        ##print(pose.pose.position.x)        
        ##print(pose.pose.position.y)
        
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.tmpposes.append(copy.deepcopy(pose))
        self.realpath.poses=self.tmpposes

        self.realpath_pub.publish(self.realpath)

    # make points homogeneous, copy them to maintain the originals

        self.tar_pc = self.laserToNumpy(msg)
        self.target_laser=self.laserEstimation(msg,self.xEst)
        self.ver_pc=self.laserToNumpy(self.target_laser)
        self.publishResult(transform_acc)
        time_1 = rospy.Time.now()
        #print("time_cost: ",time_1-time_0)
        self.laser_count =0

        self.icpcloud.header.seq = 0
        self.icpcloud.header.stamp = rospy.Time(0)
        self.icpcloud.header.frame_id = 'map'

        points=Point32()
        points.x=copy.deepcopy(self.xEst[0,0])
        points.y=copy.deepcopy(self.xEst[1,0])
        points.z=0.01
        self.icppoints.append(copy.deepcopy(points))
        #self.icppoints[self.indexi].x=copy.deepcopy(self.xEst[0,0])
        #self.icppoints[self.indexi].y=copy.deepcopy(self.xEst[1,0])
        #self.icppoints[self.indexi].z=0.01
        #print("icpp")
        #print(self.icppoints)
        mchannels=ChannelFloat32()
        mchannels.name="rgb"
        for i in range(0,len(self.icppoints)):
            mchannels.values.append(110)
        self.icpcloud.points=self.icppoints
        self.icpcloud.channels.append(mchannels)
        self.icpp_pub.publish(self.icpcloud)


        self.icpcloud2.header.seq = 0
        self.icpcloud2.header.stamp = rospy.Time(0)
        self.icpcloud2.header.frame_id = 'map'

        points2=Point32()
        points2.x=copy.deepcopy(transform_acc[0,2])
        points2.y=copy.deepcopy(transform_acc[1,2])
        points2.z=0.01
        self.icppoints2.append(copy.deepcopy(points2))
        #self.icppoints[self.indexi].x=copy.deepcopy(self.xEst[0,0])
        #self.icppoints[self.indexi].y=copy.deepcopy(self.xEst[1,0])
        #self.icppoints[self.indexi].z=0.01
        #print("icpp2")
        #print(self.icppoints2)
        mchannels=ChannelFloat32()
        mchannels.name="rgb"
        for i in range(0,len(self.icppoints)):
            mchannels.values.append(110)
        self.icpcloud2.points=self.icppoints2
        self.icpcloud2.channels.append(mchannels)
        self.icpp_pub2.publish(self.icpcloud2)
        self.indexi=self.indexi+1
        print(self.calcDist(self.xEst,self.txEst)/self.calcDist(self.xdom,self.txEst))



    def updateMap(self,msg):
        #print("debug: try update map obstacle")
        '''
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            self.map = getMap().map
            # #print(self.map)
        except:
            e = sys.exc_info()[0]
            #print('Service call failed: %s'%e)
        '''
        self.mapdata=msg.data
        # Update for planning algorithm
        self.map_data = np.array(self.mapdata).reshape((-1,msg.info.height)).transpose()#heigh=129
        tx,ty = np.nonzero((self.map_data > 20)|(self.map_data < -0.5))
        ox = (tx*msg.info.resolution+msg.info.origin.position.x)*1.0
        oy = (ty*msg.info.resolution+msg.info.origin.position.y)*1.0
        self.obstacle = np.vstack((ox,oy)).transpose()
        self.obstacle_r = msg.info.resolution
        #print("debug: update map obstacle success! ")

    def laserEstimation(self,msg,x):
        data=LaserScan()
        data=msg
        data.ranges=list(msg.ranges)
        data.intensities=list(msg.intensities)

        #data.angle_min= -3.14159011841
        #data.angle_max= 3.14159011841
        #data.angle_increment= 0.0527998320758
        for i in range(0,len(data.ranges)):
            data.ranges[i]=30
            data.intensities[i]=0.5
        #tx,ty = np.nonzero((self.map_data > 20)|(self.map_data < -0.5))
        #ox = (tx*self.map.info.resolution+self.map.info.origin.position.x-x[0,1])*1.0
        #oy = (ty*self.map.info.resolution+self.map.info.origin.position.y-x[1,2])*1.0
        ##print("shape")
        ##print(self.obstacle.shape[0])
        for i in range(0,len(self.obstacle)):
            ox=self.obstacle[i,0]-x[0,0]
            oy=self.obstacle[i,1]-x[1,0]
            #ox=self.obstacle[i,0]-0
            #oy=self.obstacle[i,1]-0
            alpha=math.atan2(oy,ox)-x[2,0]
            
            rou=math.hypot(ox,oy)
            ##print("rou"+str(rou)+"alpha"+str(alpha))
            delta_alpha=math.atan2(self.obstacle_r/2,rou)
            alpha_min=math.atan2(math.sin(alpha-delta_alpha),math.cos(alpha-delta_alpha))
            alpha_max=math.atan2(math.sin(alpha+delta_alpha),math.cos(alpha+delta_alpha))
            index_min=int(np.floor(abs((alpha_min+math.pi)/msg.angle_increment)))
            index_max=int(np.floor(abs((alpha_max+math.pi)/msg.angle_increment)))
            ##print("min"+str(index_min)+"max"+str(index_max))
            if index_min<=index_max:
                for index in range(index_min,index_max+1):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou

            else:
                #print(">>>>>>>>>>>>>>>>>>>")
                for index in range(0,index_max+1):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou               
                for index in range(index_min,len(msg.ranges)):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou 
        
        ##print(data)
        self.target_laser=data;
        return data;




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

        ##print(c_shift)
        ##print(p_shift)
        ##print("w is")
        ##print(W)
        try:
            u, s, vh = np.linalg.svd(W)
        except Exception as e:
            print(e)
            print(src)
            print(tar)

        R = (np.dot(u,vh)).T
        T = pm - (np.dot(R,cm))
        ##print("this is T2")
        ##print(T)
        return R, T


    def publishResult(self,T):


        s = self.xEst
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster_2.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")
        # odom
        aodom = Odometry()
        aodom.header.stamp = rospy.Time.now()
        aodom.header.frame_id = "world_base"

        aodom.pose.pose.position.x = s[0]
        aodom.pose.pose.position.y = s[1]
        aodom.pose.pose.position.z = 0.001
        aodom.pose.pose.orientation.x = q[0]
        aodom.pose.pose.orientation.y = q[1]
        aodom.pose.pose.orientation.z = q[2]
        aodom.pose.pose.orientation.w = q[3]
        ##print(aodom)
        self.location_pub.publish(aodom)

 

        # #print("t: ",T)
        delta_yaw = math.atan2(T[1,0],T[0,0])
        #print("sensor-delta-xyt: ",T[0,2],T[1,2],delta_yaw)
        s = self.sensor_sta
        #self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        #self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        #self.sensor_sta[2] = s[2] + delta_yaw
        self.sensor_sta[0] = T[0,2]
        self.sensor_sta[1] = T[1,2]
        self.sensor_sta[2] = delta_yaw
        #print("sensor-global: ",self.sensor_sta)

        # tf
        s = self.sensor_sta
        q = tf.transformations.quaternion_from_euler(0,0,self.sensor_sta[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_odom","world_base")
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        #print("tfsend")
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

        self.laser_pub.publish(self.target_laser)
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