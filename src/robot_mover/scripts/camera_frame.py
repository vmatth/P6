#!/usr/bin/env python  
import roslib   

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print("Broadcasting camera tf")
        br.sendTransform((0.0465, -0.5325, 1.1), #xyz
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "camera_frame",
                         "base_link")
        rate.sleep()