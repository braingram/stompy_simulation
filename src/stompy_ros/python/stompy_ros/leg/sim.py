#!/usr/bin/env python
"""
Simulated:
    - generate, send and monitor trajectories
    - monitor controller state (to track trajectories and position)
    - monitor joint sensors

Make sure the hw/sw always has 1 second worth of points

new joint states
send foot state
halt (severity)
set_plan (plan)
get_position ?
update (send more points if needed)
track current point (from sent points)
"""

import actionlib
import rospy

import control_msgs.msg
import control_msgs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg

from .. import kinematics
from .. import transforms
from . import plans


class SimLeg(object):
    def __init__(self, name):
        self.name = name
        self._pos = None
        self._points = []
        self._pgen = plans.PointGenerator()
        self._buffer_time = 1.
        self._joint_names = [
            '%s_%s' % (self.name, jn) for jn in
            ('hip', 'thigh', 'knee')]
        self._calf_name = '%s_calf' % self.name
        self.connect()

    def send_leg(self, position, load, time):
        pass

    def connect(self):
        self._trajectory_publisher = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % self.name,
            control_msgs.msg.FollowJointTrajectoryAction)
        self._joint_state_sub = rospy.Subscriber(
            '/stompy/joint_states', sensor_msgs.msg.JointState,
            self._new_joint_states)
        self._pos_pub = rospy.Publisher(
            '/stompy/%s/leg_pos' % self.name,
            geometry_msgs.msg.Point,
            queue_size=10)

    def _new_joint_states(self, msg):
        ps = {}
        load = None
        for i in xrange(len(msg.name)):
            if msg.name[i] in self._joint_names:
                ps[msg.name[i].split('_')[1]] = msg.position[i]
            elif msg.name[i] == self._calf_name:
                load = msg.position[i]
        if len(ps) != 3:
            return
        #if self._pos is None:
        #    l = [ps['hip'], ps['thigh'], ps['knee']]
        #    self._pos = plans.Point(
        #        l, transforms.identity_3d(), msg.header.stamp.to_sec(),
        #        kinematics.frames.JOINT_FRAME)
        l = [ps['hip'], ps['thigh'], ps['knee']]
        self._pos = plans.Point(
            l, transforms.identity_3d(), msg.header.stamp.to_sec(),
            kinematics.frames.JOINT_FRAME)

        foot = kinematics.frames.convert(
            [ps[j] for j in ('hip', 'thigh', 'knee')],
            kinematics.frames.JOINT_FRAME,
            kinematics.frames.BODY_FRAME)
        # calculate load
        # convert to newtons then to lbs
        load = load * 105075.9 * 0.224808942443
        self.send_leg(foot, load, msg.header.stamp)

    def halt(self, severity):
        # TODO handle severity
        self._trajectory_publisher.cancel_goal()
        # TODO send 0 move
        #self.points.drop()

    def add_plan(self, plan):
        # set start time
        if plan.start_time is None:
            plan.start_time = rospy.Time.now().to_sec()
        # TODO special handling of mode?
        # drop points after the plan start time
        drop_inds = []
        for (i, p) in enumerate(self._points):
            if p.timestamp > plan.start_time:
                drop_inds.append(i)
        for i in drop_inds[::-1]:
            self._points.pop(i)
        # TODO special handling of frame?
        #if plan.frame != kinematics.frames.JOINT_FRAME:
        #    print("Non joint frame not implemented")
        #    return
        print("Adding plan: %s" % plan)
        self._pgen.add_plan(plan)

    def get_position(self, frame):
        if self._pos is None:
            return None, None
        return (
            kinematics.frames.convert(
                self._pos.position,
                kinematics.frames.JOINT_FRAME, frame), self._pos.timestamp)

    def send_points(self, ps):
        msg = control_msgs.msg.FollowJointTrajectoryGoal()
        for jn in self._joint_names:
            msg.trajectory.joint_names.append(jn)
        st = ps[0].timestamp
        print("building trajectory starting at: %s" % st)
        for p in ps:
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration(p.timestamp - st)
            pt.positions = p.position
            msg.trajectory.points.append(pt)
        msg.trajectory.header.stamp = rospy.Time(st)
        print("publishing trajectory")
        self._trajectory_publisher.send_goal(msg)

    def update(self):
        if self._pos is None:
            # can't update until an initial position is found
            return
        t = rospy.Time.now().to_sec()
        # update position
        drop_inds = []
        for (i, p) in enumerate(self._points):
            if p.timestamp < t:
                drop_inds.append(i)
        #if len(drop_inds):
        #    self._pos = self._points[drop_inds[-1]]
        #    msg = geometry_msgs.msg.Point()
        #    msg.x = self._pos.position[0]
        #    msg.y = self._pos.position[1]
        #    msg.z = self._pos.position[2]
        #    self._pos_pub.publish(msg)
        msg = geometry_msgs.msg.Point()
        msg.x = self._pos.position[0]
        msg.y = self._pos.position[1]
        msg.z = self._pos.position[2]
        self._pos_pub.publish(msg)
        # generate more points?
        ps = []
        if (
                len(self._points) == 0 or
                self._points[-1].timestamp < t):
            # generate points from position at the plan start time
            print(
                "%s: generating points from plan start: %s" % (
                    t, self._pos))
            ps = self._pgen.generate_points(
                100, self._pos, use_plan_start=t)
        elif (self._points[-1].timestamp - t < self._buffer_time):
            print(
                "%s: generating points from last point: %s, %s" % (
                    t, self._points[-1], self._pos))
            #self._pgen.generate_points
            ps = self._pgen.generate_points(100, self._points[-1])
        if len(ps):
            self._points.extend(ps)
            print("sending: len(ps) = %s" % (len(ps), ))
            self.send_points(ps)
        # drop old points
        for i in drop_inds[::-1]:
            self._points.pop(i)
