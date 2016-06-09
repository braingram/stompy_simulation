#!/usr/bin/env python

import sys

import rospy
import std_msgs.msg


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
        p = rospy.Publisher(tn, std_msgs.msg.Float64, queue_size=10)
        ps[tn] = p

#pub = lambda lc, v: ps[lc].publish(fm(v))
pub = lambda leg, joint, v: ps[lc(leg, joint)].publish(v)

td = 0.005
tmax = 0.6981
tmin = 0
tp = tmin

kd = -0.005
kmax = 0.
kmin = -0.6981
kp = kmax

hd = 0.01
hmax = 0.2
hmin = -0.2
hp = hmin


def stand():
    for l in legs:
        pub(l, 'thigh', 0.6981)
        pub(l, 'knee', -0.6981)
        pub(l, 'hip', 0.)

#r = rospy.Rate(10)
while not rospy.is_shutdown():
    stand()
    rospy.sleep(1.)
