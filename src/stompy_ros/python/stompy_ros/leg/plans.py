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
TARGET_MODE = 3  # move to lx, ly, lz

mode_names = {
    'stop': STOP_MODE,
    'velocity': VELOCITY_MODE,
    'arc': ARC_MODE,
    'target': TARGET_MODE,
}
modes = mode_names.values()


def lookup_mode(mode):
    if mode in modes:
        return mode
    if mode in mode_names:
        return mode_names[mode]
    raise ValueError("Unknown mode: %s" % mode)


def id_gen(start=0):
    while True:
        yield start
        start += 1


class Point(object):
    """
    position : always in joint frame
    transform : might be in a different frame
    """
    _pid_gen = id_gen()

    def __init__(self, position, transform, timestamp, frame):
        self.position = position
        self.transform = transform
        self.timestamp = timestamp
        self.frame = frame
        #self.mode = mode
        self.pid = self._pid_gen.next()

    def __repr__(self):
        return "Point(%s, %s, %s, %s, %s)" % (
            self.pid, self.timestamp, self.position, self.transform,
            self.frame)

    def project(self, dt):
        # convert from joint frame to point frame
        p = kinematics.frames.convert(
            self.position, kinematics.frames.JOINT_FRAME,
            self.frame)
        # TODO scale transform by dt
        tp = transforms.transform_3d(
            self.transform, *p)
        # transform back to joint frame
        return kinematics.frames.convert(
            tp, self.frame, kinematics.frames.JOINT_FRAME)


class Plan(object):
    def __init__(self, mode, frame, target, start_time, speed=None):
        # self.transform
        # self.start_time
        self.mode = mode
        self.frame = frame
        self.target = target
        self.start_time = start_time
        if hasattr(self.start_time, 'to_sec'):
            self.start_time = self.start_time.to_sec()
        self.speed = speed
        self.transform = self._to_transform()

    def __repr__(self):
        return "Plan(%s, %s, %s, %s, %s)" % (
            self.start_time, self.mode, self.frame,
            self.target, self.transform)

    def _to_transform(self):
        if self.mode == STOP_MODE:
            return transforms.identity_3d()
        elif self.mode == VELOCITY_MODE:
            return transforms.translation_3d(*self.target)
        elif self.mode == ARC_MODE:
            return transforms.rotation_about_point_3d(
                *self.target)
        else:
            raise ValueError("Unknown mode: %s" % self.mode)


class PointGenerator(object):
    """
    pgen.add_plan(plan)
    pgen.generate_points(start_point, 100, 0.01)  # n, dt
    # drop points
    """
    def __init__(self, blend_time=0.5):
        self.plans = []
        self.blend_time = blend_time

    def add_plan(self, plan):
        # TODO handle blending multiple frames?
        # or just drop plans
        #drop_plans = False
        #for p in self.plans:
        #    if p.frame != plan.frame:
        #        drop_plans = True
        #if drop_plans:
        #    self.plans = []
        self.plans.append(plan)
        self.plans = sorted(self.plans, key=lambda p: p.start_time)
        print("Updated plans: %s" % self.plans)
        #return not drop_plans

    def plan_at_t(self, t):
        for i in xrange(len(self.plans)):
            if self.plans[i].start_time < t:
                if i == len(self.plans)-1:
                    return self.plans[i]
                if self.plans[i + 1].start_time > t:
                    return self.plans[i]
        return None

    def previous_plan(self, plan):
        if plan is None:
            return None
        pi = self.plans.index(plan)
        if pi == 0:
            return None
        else:
            return self.plans[pi-1]

    def generate_points(self, n, start_point, dt=0.01, use_plan_start=False):
        # the points hold the transform, always blend
        p = start_point
        ps = []
        if use_plan_start is not False:
            if len(self.plans) == 0:
                return []
            if use_plan_start is True:
                t = self.plans[0].start_time
            else:  # use_plan_start is a time
                plan = self.plan_at_t(use_plan_start)
                if plan is None:
                    t = None
                else:
                    t = plan.start_time
                #t = self.plan_at_t(use_plan_start).start_time
                # if no plan before use_plan_start, then use next
                if t is None or use_plan_start - t < (dt * n):
                    for plan in self.plans:
                        if plan.start_time >= use_plan_start:
                            t = plan.start_time
                            break
                    print("Using plan in future: %s" % t)
                if t is None:
                    print("Found no plans at %s" % use_plan_start)
                    return []
        else:
            t = p.timestamp
        if hasattr(t, 'to_sec'):
            t = t.to_sec()
        # drop old plans
        drop_inds = []
        if len(self.plans) > 1:
            for i in xrange(len(self.plans)-1):
                plan = self.plans[i]
                # np = self.plans[i+1]
                if (
                        plan.start_time < t and
                        t - plan.start_time > (self.blend_time * 2)):
                    drop_inds.append(i)
        for i in drop_inds[::-1]:
            dp = self.plans.pop(i)
            print("Dropped old plan: %s" % dp)
        # if no plans generate no points
        if len(self.plans) == 0:
            return []
        # TODO what to do if all plans are STOP?
        for i in xrange(n):
            t += dt
            plan = self.plan_at_t(t)
            # TODO what about if plan is None?
            if plan is None:
                continue
            # TODO what to do when mode == STOP
            #previous = self.previous_plan(plan)
            #if previous is None or t > (plan.start_time + self.blend_time):
            #    # no blending
            ptdt = t - p.timestamp
            pldt = t - plan.start_time
            #location = p.project(ptdt)
            location = p.project(ptdt)  # TODO account for dt
            #print(p.location, location)
            # blend transforms
            if p.frame != plan.frame:
                # TODO convert point transform to plan frame
                if not (
                        plan.mode == STOP_MODE or
                        transforms.is_stop_3d(p.transform)):
                    raise Exception(
                        "%s" % (p.frame, plan.frame, plan.mode, p.transform))
            if pldt > self.blend_time:
                transform = plan.transform
            else:
                w = pldt / self.blend_time
                transform = (
                    p.transform * (1 - w) +
                    plan.transform * w)
                #transform = (
                #    p.transform +
                #    (plan.transform - p.transform) *
                #    pdt / self.blend_time)
            #print(
            #    plan.transform, p.transform, transform,
            #    p.location, p.project(dt))
            # TODO mode?
            p = Point(
                location, transform, t,
                plan.frame)
            #p = Point(
            #    location, transform, p.timestamp + dt,
            #    kinematics.frames.JOINT_FRAME)
            ps.append(p)
        return ps


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
    elif mode == TARGET_MODE:
        raise NotImplementedError("Target mode not yet implemented")
    else:
        raise ValueError("Unknown mode: %s" % mode)


def from_message(msg):
    mode = msg.mode
    frame = msg.frame
    target = resolve_target(msg.target, mode)
    start_time = (
        None if msg.start_time.is_zero()
        else msg.start_time.to_sec())
    speed = None if msg.speed == 0. else msg.speed
    return Plan(
        mode, frame, target, start_time, speed)


def make_message(mode, frame, target, start_time=0, speed=0):
    msg = stompy_msgs.msg.LegPlan()
    msg.mode = lookup_mode(mode)
    msg.frame = kinematics.frames.lookup_frame(frame)
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
    msg.start_time = start_time
    msg.speed = speed
    return msg


def make_stop_message(start_time=0, frame=kinematics.frames.JOINT_FRAME):
    return make_message(
        STOP_MODE, frame, None, start_time=start_time)


def test_pgen(t=None, n=0):
    if t is None:
        t = [100., 0., 0.]
    pg = PointGenerator()
    pl = Plan(VELOCITY_MODE, kinematics.frames.JOINT_FRAME, t, 0, None)
    pt = Point(
        [0, 0, 0], transforms.identity_3d(), 0, kinematics.frames.JOINT_FRAME)
    pg.add_plan(pl)
    ps = pg.generate_points(100, pt, use_plan_start=False)
    for i in xrange(n):
        ps.extend(pg.generate_points(100, ps[-1]))
    return ps
