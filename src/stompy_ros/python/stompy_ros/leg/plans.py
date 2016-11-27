#!/usr/bin/env python
"""
Leg movement plan

"""

import rospy

from .. import kinematics
from .. import transforms


STOP_MODE = 0
VELOCITY_MODE = 1  # move along vector lx, ly, lz,
ARC_MODE = 2  # rotate angular about point linear
#TARGET_MODE = 3  # move to lx, ly, lz

modes = [STOP_MODE, VELOCITY_MODE, ARC_MODE]


class Waypoint(object):
    def __init__(self, position, time, pid, transform):
        self.position = position
        self.time = time
        self.pid = pid
        self.transform = transform


class Plan(object):
    def __init__(self, mode, frame, target, start_time, speed=None):
        self.mode = mode
        self.frame = frame
        self.target = target
        self.start_time = start_time
        self.speed = speed

    def to_transform(self):
        if self.mode == STOP_MODE:
            return transforms.identity_3d()
        elif self.mode == VELOCITY_MODE:
            return transforms.translation_3d(*self.target)
        elif self.mode == ARC_MODE:
            return transforms.rotation_about_point_3d(
                *self.target)
        else:
            raise ValueError("Unknown mode: %s" % self.mode)

    def point_generator(self, start, time, pid=0, in_frame=None):
        if in_frame is None:
            in_frame = self.frame
        t = self.to_transform()
        x, y, z = start
        dt = rospy.Duration(0.1)
        while True:
            x, y, z = transforms.transform_3d(t, x, y, z)
            time += dt
            px, py, pz = kinematics.frames.convert(
                (x, y, z), self.frame, in_frame)
            yield Waypoint((px, py, pz), time, pid, t)
            pid += 1


def resolve_target(target, mode):
    if mode == STOP_MODE:
        return None
    elif mode == VELOCITY_MODE:
        return [target.linear.x, target.linear.y, target.linear.z]
    elif mode == ARC_MODE:
        return [
            target.linear.x, target.linear.y, target.linear.z,
            target.angular.x, target.angular.y, target.angular.z,
        ]
    else:
        raise ValueError("Unknown mode: %s" % mode)


def from_message(msg):
    mode = msg.mode
    frame = msg.frame
    target = resolve_target(msg.target, mode)
    start_time = (
        rospy.Time.now() if msg.start_time.is_zero()
        else msg.start_time)
    speed = None if msg.speed == 0. else msg.speed
    return Plan(
        mode, frame, target, start_time, speed)
