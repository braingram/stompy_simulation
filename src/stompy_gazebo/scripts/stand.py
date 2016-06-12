#!/usr/bin/env python

import sys

import rospy
import std_msgs.msg

import leg


# hip: -1.57, 1.57
# thigh: 0, 0.6981
# knee: -0.6981, 0

lc = lambda leg, joint: '/stompy/%s_%s_controller/command' % (leg, joint)
fm = lambda data: std_msgs.msg.Float64(data)
legs = ('fr', 'fl', 'mr', 'ml', 'rr', 'rl')
joints = ('hip', 'thigh', 'knee')

rospy.init_node('stompy_stand')

argv = rospy.myargv(argv=sys.argv)

# setup publishers
ps = {}
for l in legs:
    for j in joints:
        tn = lc(l, j)
        print("Publishing to: %s" % tn)
        p = rospy.Publisher(tn, std_msgs.msg.Float64, queue_size=18)
        ps[tn] = p

#pub = lambda lc, v: ps[lc].publish(fm(v))
pub = lambda leg, joint, v: ps[lc(leg, joint)].publish(v)


def stand(z):
    for l in legs:
        h, t, k = leg.compute_angles(1.1, 0.0, z)
        pub(l, 'hip', h)
        pub(l, 'thigh', t)
        pub(l, 'knee', k)

z = 0.2
#r = rospy.Rate(10)
while not rospy.is_shutdown():
    stand(z)
    if z < 1.2:
        print("z: %s" % z)
        z += 0.005
    rospy.sleep(0.1)
