#!/usr/bin/env python

import math
import sys

import rospy
import std_msgs.msg


compute = False
# meters
hip_link = 0.279  # coxa
#thigh_link = 1.37158  # femer
thigh_link = 1.409  # femer
#calf_link = 1.49589  # tibia, varies with load
# TODO get a better estimate of this
calf_link = 2.  # tibia, varies with load

# xy looking down
x = 0
y = 0
z = 0

x = float(sys.argv[1])
y = float(sys.argv[2])
z = float(sys.argv[3])
z_offset = 0.2
if len(sys.argv) > 4:
    compute = True


def compute_angles(x, y, z):
    """
    hip: + tilts down
    knee: - tilts out
    """
    z_offset = z
    # distance from hip to ground
    l = math.sqrt(x * x + y * y)

    hip_angle = math.atan2(y, x)

    L = math.sqrt(z_offset * z_offset + (l - hip_link) ** 2.)
    thigh_angle = (
        math.acos(z_offset / L) +
        math.acos(
            (calf_link ** 2 - thigh_link ** 2 - L ** 2) /
            (-2 * thigh_link * L)))
    thigh_angle = math.pi - thigh_angle

    knee_angle = math.acos(
        (L ** 2 - calf_link ** 2 - thigh_link ** 2) /
        (-2 * calf_link * thigh_link))
    knee_angle = -knee_angle

    print hip_angle, thigh_angle, knee_angle
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

dx = 0.01
dy = 0.0
dz = 0.0
while not rospy.is_shutdown():
    hp.publish(hip_angle)
    tp.publish(thigh_angle)
    kp.publish(knee_angle)
    x += dx
    y += dy
    z += dz
    if compute:
        hip_angle, thigh_angle, knee_angle = compute_angles(x, y, z)
    rospy.sleep(0.1)
