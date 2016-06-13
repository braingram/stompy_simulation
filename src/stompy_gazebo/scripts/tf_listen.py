#!/usr/bin/env python

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('tf_listen', anonymous=True)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            rt = rospy.Time.now()
            listener.waitForTransform(
                'stompyleg__fl__hip', 'stompyleg__fl__foot',
                rt, rospy.Duration(1.0))
            (t, r) = listener.lookupTransform(
                'stompyleg__fl__hip', 'stompyleg__fl__foot',
                rt)
            print t, r
        except tf.LookupException:
            pass
        rospy.sleep(1.0)
