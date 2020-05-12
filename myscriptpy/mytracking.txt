#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from threading import Lock,Thread
import math
import time
class Tracking:
    def __init__(self):
        self.arrive_threshold = 0.2
        self.vx = 0.0
        self.vw = 0.0

        self.lock = Lock()
        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.tracking_thread = None
        pass
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x-self.x,p.y-self.y)
        if dis < self.arrive_threshold and self.goal_index < len(self.path.poses)-1:
            self.goal_index = self.goal_index + 1
        self.midpose_pub.publish(self.path.poses[self.goal_index])

    def pathCallback(self,msg):
        print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release()
        if self.tracking_thread == None:
            self.tracking_thread = Thread(target=self.trackThreadFunc)
            self.tracking_thread.start()
        pass
    def initTracking(self):
        self.goal_index = 2
        self.updateGlobalPose()
        pass
    def trackThreadFunc(self):
        print("running track thread!!")
        # while self.plan_lastIndex > self.plan_target_ind:
        while True:
            self.planOnce()
            time.sleep(0.001)
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        self.tracking_thread = None
        pass
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()

        target = self.path.poses[self.goal_index].pose.position
        k1=0.3
        k2=1
        dx = target.x - self.x
        dy = target.y - self.y
        
        if(self.yaw<0):
            self.yaw=2*3.14159+self.yaw
        distp = math.hypot(dx,dy)
        target_angle = math.atan2(dy, dx)
        if(target_angle<0):
            target_angle=2*3.14159+target_angle
        target_theta=(target_angle-self.yaw)
        if target_theta>3.14159:
            target_theta-=2*3.14159
        garma=(k2*(target_theta-math.atan(k1*target_angle))+(1+k1/(1+(k1*target_angle)**2))*math.sin(target_theta))
        qulv=-garma/distp
        k3=2
        self.vx = k3*distp
        self.vw = k3*garma
        rospy.loginfo("x: "+str(self.x)+" y:"+str(self.y))
        rospy.loginfo("mgoalx: "+str(target.x)+" mgoaly:"+str(target.y))
        rospy.loginfo("dx: "+str(dx)+" dy:"+str(dy))
        rospy.loginfo("this is data")
        rospy.loginfo("target "+str(target_angle))
        rospy.loginfo("yaw "+str(self.yaw))
        rospy.loginfo("vw "+str(self.vw))
        rospy.loginfo("vx "+str(self.vx))     
        if self.vw>2:
            self.vw=2
            self.vx=0.5
        if self.vw<-2:
            self.vw=-2
            self.vx=0.5
        if self.vx>1:
            self.vx=1
        self.publishVel()

        self.lock.release()
        pass
    def publishVel(self,zero = False):
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('stupid_tracking')
    t = Tracking()
    rospy.spin()

if __name__ == '__main__':
    main()