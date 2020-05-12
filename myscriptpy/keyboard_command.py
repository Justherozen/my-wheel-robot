#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 
import sys, select, termios, tty


moveBindings = {
        'w':(1,0,0),
        'a':(0,1,0),
        'd':(0,-1,0),
        's':(-1,0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub1 = rospy.Publisher('/course_agv/velocity', Twist, queue_size = 1)
    rospy.init_node('keyboard_command')
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    th = 0
    status = 0

    try:
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
           
            else:
                x = 0
                y = 0
                th = 0
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub1.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub1.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
