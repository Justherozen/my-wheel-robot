#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from icp import ICP
from ekf import EKF
import sys
class Localization():
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()
        
        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.isFirstScan_2 = True
        self.src_pc = []
        self.tar_pc = []
        self.map_data=[]
        # State Vector [x y yaw]
        self.xOdom = np.zeros((3,1))
        self.xEst = np.zeros((3,1))
        self.xEKF=np.zeros((4,1))
        self.PEst = np.eye(4)
        self.lasttime=0
        self.thistime=0
        self.laser_count=0
        # map observation
        self.obstacle = []
        self.target_laser=LaserScan()
        # radius
        self.obstacle_r = 10
        self.laser_obstacle=[]
        self.veloacc=None
        self.transform_acc =None
        # init map
        self.updateMap()
        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster = tf.TransformBroadcaster()
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        self.tf = tf.TransformListener()
        
    def updateMap(self):
        print("debug: try update map obstacle")
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            self.map = getMap().map
            # print(self.map)
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        self.map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()#heigh=129
        tx,ty = np.nonzero((self.map_data > 20)|(self.map_data < -0.5))
        ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        self.obstacle = np.vstack((ox,oy)).transpose()
        self.obstacle_r = self.map.info.resolution
        print("debug: update map obstacle success! ")

    def laserCallback(self,msg):
        if(msg.ranges[0]>999 or msg.ranges[3]>999  or msg.ranges[3]<-999  or msg.ranges[0]<-999):
            return 
        #print("msg")
        #print(msg)
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
             self.lasttime=msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000
             self.isFirstScan = False
             self.laser_count = 0
             return
        
        # process every 5 laser scan for laser fps too high
        self.laser_count += 1
        if self.laser_count <= 20:
            return

        self.laser_count = 0
        #self.ekf.DT=msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000-self.lasttime;
        #print("DT")
        #print(self.lasttime)
        tmp1=self.calc_odometry(msg)
        if self.veloacc is None:
            self.veloacc=tmp1
        else:
            self.veloacc=np.dot(self.veloacc,tmp1)

        #u=np.zeros((2,1))
        #u[0,0]=math.sqrt(veloacc[0,2]**2+veloacc[1,2]**2)/self.ekf.DT
        #u[1,0]=math.atan2(veloacc[1,0],veloacc[0,0])/self.ekf.DT 
        tmp2=self.calc_map_observation(msg,self.xOdom)
        if self.transform_acc is None:
            self.transform_acc=tmp2
        else:     
            self.transform_acc=np.dot(self.transform_acc,tmp2)
        
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.xEst[0] = self.trans[0]+np.random.uniform(-1,1,1)*0.15
        self.xEst[1] = self.trans[1]+np.random.uniform(-1,1,1)*0.15
        self.xEst[2] = yaw+np.random.uniform(-1,1,1)*0.1
        print("velo")
        print(self.veloacc)
        print("transfrom_acc")
        print(self.transform_acc)
        #self.xEst[0]=self.transform_acc[0,2]
        #self.xEst[1]=self.transform_acc[1,2]
        #self.xEst[2]=math.atan2(self.transform_acc[1,0],self.transform_acc[0,0])
        self.xOdom[0]=self.veloacc[0,2]
        self.xOdom[1]=self.veloacc[1,2]
        self.xOdom[2]=math.atan2(self.veloacc[1,0],self.veloacc[0,0])
        #z=np.ones((1,2))
        #z[0,0]=self.xEst[0]+math.cos(self.xEst[2])*transform_acc[0,2]-math.sin(self.xEst[2])*transform_acc[1,2]
        #z[0,1]=self.xEst[1]+math.sin(self.xEst[2])*transform_acc[0,2]+math.cos(self.xEst[2])*transform_acc[1,2]
     #self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        #self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        #self.sensor_sta[2] = s[2] + delta_yaw
        self.xEKF[0,0]=self.xEst[0,0]
        self.xEKF[1,0]=self.xEst[1,0]
        self.xEKF[2,0]=self.xEst[2,0]
        self.xEKF[3,0]=0
        #self.xEKF, self.PEst=self.ekf.estimate(self.xEKF, self.PEst, z, u)
        #self.xEst[0:3,:]=self.xEKF[0:3,:]
       # self.lasttime=msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000
        self.publishResult()
        pass
    
    def laserEstimation(self,msg,x):
        data=LaserScan()
        data.ranges=list(msg.ranges)
        data.intensities=list(msg.intensities)
        data.header=msg.header
        data.angle_min= -3.14159011841
        data.angle_max= 3.14159011841
        data.angle_increment= 0.0527998320758
        data.header.stamp = rospy.Time.now()
        for i in range(0,len(data.ranges)):
            data.ranges[i]=30
            data.intensities[i]=0
        #tx,ty = np.nonzero((self.map_data > 20)|(self.map_data < -0.5))
        #ox = (tx*self.map.info.resolution+self.map.info.origin.position.x-x[0,1])*1.0
        #oy = (ty*self.map.info.resolution+self.map.info.origin.position.y-x[1,2])*1.0
        #print("shape")
        #print(self.obstacle.shape[0])
        for i in range(0,self.obstacle.shape[0]):
            ox=self.obstacle[i,0]-x[0,0]
            oy=self.obstacle[i,1]-x[1,0]
            #ox=self.obstacle[i,0]-0
            #oy=self.obstacle[i,1]-0
            alpha=math.atan2(oy,ox)
            
            rou=math.hypot(ox,oy)
            #print("rou"+str(rou)+"alpha"+str(alpha))
            delta_alpha=math.atan2(self.obstacle_r/2,rou)
            alpha_min=math.atan2(math.sin(alpha-delta_alpha),math.cos(alpha-delta_alpha))
            alpha_max=math.atan2(math.sin(alpha+delta_alpha),math.cos(alpha+delta_alpha))
            index_min=int(np.floor(abs((alpha_min-msg.angle_min)/msg.angle_increment)))
            index_max=int(np.floor(abs((alpha_max-msg.angle_min)/msg.angle_increment)))
            #print("min"+str(index_min)+"max"+str(index_max))
            if index_min<=index_max:
                for index in range(index_min,index_max+1):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou

            else:
                for index in range(0,index_max+1):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou               
                for index in range(index_min,len(msg.ranges)):
                    if data.ranges[index]>rou:
                        data.ranges[index]=rou 
        
        #print(data)
        self.target_laser=data;
        return data;


    def calc_map_observation(self,msg,x):
        mylaser_data=self.laserEstimation(msg,x)
        src_data=self.laserToNumpy(mylaser_data)
        #tmp_data=np.ones((3,1))
        #tmp_data[0,0]=999
        #tmp_data[1,0]=999
        #tmp_data[2,0]=1
        tar_data=self.laserToNumpy(msg)
        #tmp_data=np.repeat(tmp_data,-tar_data.shape[1]+src_data.shape[1],axis=1)
        #print(tar_data.shape)#120
        #tar_data=np.hstack((tar_data,tmp_data))
        #print(tar_data.shape)#3864
        #print(tmp_data.shape)
        #print(src_data.shape)
        transform_acc=self.icp.process(tar_data,src_data)
        return transform_acc

    def calc_odometry(self,msg):
        if self.isFirstScan_2:
            self.tar_pc = self.laserToNumpy(msg)
            self.isFirstScan_2 = False
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
             
        self.src_pc = self.laserToNumpy(msg)
        #print(self.tar_pc)
        transform_acc = self.icp.process(self.src_pc,self.tar_pc)
        self.tar_pc = self.laserToNumpy(msg)
        return transform_acc

    def laserToNumpy(self,msg):
        #print(msg.ranges)
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def publishResult(self):
        # tf
        s = self.xEst
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")
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

        self.location_pub.publish(odom)
        

        s = self.xOdom
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        #odom
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
        self.map_broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0, 0,0),
                            rospy.Time.now(),"world_base","map")
        pass
def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
