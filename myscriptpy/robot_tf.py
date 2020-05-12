#!/usr/bin/env python  
import roslib
import rospy
import tf
import math
from gazebo_msgs.msg import LinkStates
def callback(data):
    br = tf.TransformBroadcaster()   
    msg=data.pose[data.name.index("course_agv::robot_base")];
    br.sendTransform((msg.position.x, msg.position.y, 0),
                        (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),
                        rospy.Time.now(),
                        "robot_base",
                        "world_base"
                        )

if __name__ == '__main__':
    rospy.init_node('robot_tf')
    rospy.Subscriber("/gazebo/link_states",LinkStates,callback)
    rospy.spin()
