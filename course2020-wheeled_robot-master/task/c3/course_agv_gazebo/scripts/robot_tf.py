#!/usr/bin/env python  
import roslib
roslib.load_manifest('course_agv_gazebo')
import rospy
import tf
import turtlesim.msg
import gazebo_msgs
def callback(data):
    br = tf.TransformBroadcaster()
    if (data.name=="course_agv::robot_base"):
        msg=data.pose;
        br.sendTransform((msg.position.x, msg.position.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 2*math.acos(msg.orientation.w) ),
                        rospy.Time.now(),
                        "robot_base",
                        "world_base")

if __name__ == '__main__':
    rospy.init_node('robot_tf')
    ("/gazebo/link_states",gazebo_msgs.LinkStates,callback)
    rospy.spin()