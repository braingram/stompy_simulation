#!/usr/bin/env python
"""
Leg movement plan

"""

import rospy

from .. import kinematics
from .. import transforms

import stompy_msgs.msg


STOP_MODE = 0
VELOCITY_MODE = 1  # move along vector lx, ly, lz,
ARC_MODE = 2  # rotate angular about point linear
#TARGET_MODE = 3  # move to lx, ly, lz

mode_names = {
    'stop': STOP_MODE,
    'velocity': VELOCITY_MODE,
    'arc': ARC_MODE,
}
modes = mode_names.values()


def lookup_mode(mode):
    if mode in modes:
        return mode
    if mode in mode_names:
        return mode_names[mode]
    raise ValueError("Unknown mode: %s" % mode)


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

    def point_generator(self, start, time, pid=0, in_frame=None, dt=0.1):
        if in_frame is None:
            in_frame = self.frame
        t = self.to_transform()
        x, y, z = start
        dt = rospy.Duration(dt)
        # TODO check limits
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
    mode = msg.mode.data
    frame = msg.frame.data
    target = resolve_target(msg.target, mode)
    start_time = (
        None if msg.start_time.data.is_zero()
        else msg.start_time.data)
    speed = None if msg.speed.data == 0. else msg.speed.data
    return Plan(
        mode, frame, target, start_time, speed)


def make_message(mode, frame, target, start_time=0, speed=0):
    msg = stompy_msgs.msg.LegPlan()
    msg.mode.data = lookup_mode(mode)
    msg.frame.data = kinematics.frames.lookup_frame(frame)
    if mode == STOP_MODE:
        pass
    elif mode == VELOCITY_MODE:
        msg.target.linear.x = target[0]
        msg.target.linear.y = target[1]
        msg.target.linear.z = target[2]
    elif mode == ARC_MODE:
        msg.target.linear.x = target[0]
        msg.target.linear.y = target[1]
        msg.target.linear.z = target[2]
        msg.target.angular.x = target[3]
        msg.target.angular.y = target[4]
        msg.target.angular.z = target[5]
    if not isinstance(start_time, rospy.rostime.Time):
        start_time = rospy.Time(start_time)
    msg.start_time.data = start_time
    msg.speed.data = speed
    return msg


def make_stop_message(start_time=0):
    return make_message(
        STOP_MODE, kinematics.frames.JOINT_FRAME, None, start_time=start_time)
