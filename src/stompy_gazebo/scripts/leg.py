#!/usr/bin/env python

import argparse
import numpy

import rospy
import std_msgs.msg


compute = False
# meters
hip_limits = (-1.57079, 1.57079)
hip_link = 0.279  # coxa
#thigh_link = 1.37158  # femer
thigh_limits = (0., 0.6981)
thigh_link = 1.409  # femer
#calf_link = 1.49589  # tibia, varies with load
# TODO get a better estimate of this
knee_limits = (-0.6981, 0.)
# calf_link = 1.77
# ankle = 0.0412
# foot = 0.1 ?
# 1.77 + 0.0412 + 0.1 = 1.9112
calf_link = 1.9112  # tibia, varies with load

p = argparse.ArgumentParser()
p.add_argument('-x', '--x', default=0., type=float)
p.add_argument('-y', '--y', default=0., type=float)
p.add_argument('-z', '--z', default=0., type=float)
p.add_argument('-X', '--dx', default=0., type=float)
p.add_argument('-Y', '--dy', default=0., type=float)
p.add_argument('-Z', '--dz', default=0., type=float)
p.add_argument('-r', '--raw', default=False, action='store_true')
args = p.parse_args()
# xy looking down
x = args.x
y = args.y
z = args.z

z_offset = 0.2
compute = not args.raw


def compute_angles(x, y, z):
    """
    hip: + tilts down
    knee: - tilts out
    """
    z_offset = z
    # distance from hip to ground
    l = numpy.sqrt(x * x + y * y)

    hip_angle = numpy.arctan2(y, x)

    L = numpy.sqrt(z_offset * z_offset + (l - hip_link) ** 2.)
    thigh_angle = (
        numpy.arccos(z_offset / L) +
        numpy.arccos(
            (calf_link ** 2 - thigh_link ** 2 - L ** 2) /
            (-2 * thigh_link * L)))
    thigh_angle = numpy.pi - thigh_angle

    knee_angle = numpy.arccos(
        (L ** 2 - calf_link ** 2 - thigh_link ** 2) /
        (-2 * calf_link * thigh_link))
    knee_angle = -knee_angle
    #knee_angle = knee_angle + knee_limits[0]

    return hip_angle, thigh_angle, knee_angle


lc = lambda joint: '/stompyleg/%s_controller/command' % (joint, )
fm = lambda data: std_msgs.msg.Float64(data)

if compute:
    hip_angle, thigh_angle, knee_angle = compute_angles(x, y, z)
else:
    hip_angle = x
    thigh_angle = y
    knee_angle = z

rospy.init_node('stompyleg_move', anonymous=True)
hp = rospy.Publisher(lc('hip'), std_msgs.msg.Float64, queue_size=18)
tp = rospy.Publisher(lc('thigh'), std_msgs.msg.Float64, queue_size=18)
kp = rospy.Publisher(lc('knee'), std_msgs.msg.Float64, queue_size=18)

dx = args.dx
dy = args.dy
dz = args.dz
while not rospy.is_shutdown():
    print(
        "Hip: %s, Thigh: %s, Knee: %s" % (hip_angle, thigh_angle, knee_angle))
    hp.publish(hip_angle)
    tp.publish(thigh_angle)
    kp.publish(knee_angle)
    x += dx
    y += dy
    z += dz
    if compute:
        hip_angle, thigh_angle, knee_angle = compute_angles(x, y, z)
    else:
        hip_angle = x
        thigh_angle = y
        knee_angle = z
    at_limit = False
    if (hip_angle < hip_limits[0] or hip_angle > hip_limits[1]):
        print("Hit hip limit")
        at_limit = True
    if (thigh_angle < thigh_limits[0] or thigh_angle > thigh_limits[1]):
        print("Hit thigh limit")
        at_limit = True
    if (knee_angle < knee_limits[0] or knee_angle > knee_limits[1]):
        print("Hit knee limit")
        at_limit = True
    if at_limit:
        dx *= -1
        dy *= -1
        dz *= -1
        x += dx
        y += dy
        z += dz

    rospy.sleep(0.1)
