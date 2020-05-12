#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 
import sys, select, termios, tty
data1=0
data2=0
def callback(twist):
    pub1 = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command', Float64, queue_size = 1)
    pub2 = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command', Float64, queue_size = 1)
    cmd1=Float64()
    cmd2=Float64()
    global data1
    data1 +=(3*(twist.linear.x+twist.angular.z*0.5))
    global data2
    data2 +=(3*(twist.linear.x-twist.angular.z*0.5))
    cmd1=data1
    cmd2=data2
    pub1.publish(cmd1)
    pub2.publish(cmd2)
def main():
    rospy.init_node('kinematics',anonymous=True)
    rospy.Subscriber("/course_agv/velocity",Twist,callback)
    rospy.spin()
   
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
