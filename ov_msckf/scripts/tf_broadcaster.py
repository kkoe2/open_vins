#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast_transform():
    rospy.init_node("tf_broadcaster")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)  # 10 Hz
    while not rospy.is_shutdown():
        # Publish a static transform
        br.sendTransform((0, 0, 0),  # Translation
                         (0, 0, 0, 1),  # Quaternion (Identity)
                         rospy.Time.now(),
                         "pvc_salto",  # Child frame
                         "global")  # Parent frame
        rate.sleep()

if __name__ == "__main__":
    broadcast_transform()
