#!/usr/bin/env python
"""
Leg controller does the actual communication with the
leg hardware/software
Both:
    - convert plans to trajectories
    - compute joint states [sensor -> angle]
    - compute foot positions [angle -> foot -> body]
Simulated:
    - generate, send and monitor trajectories
    - monitor controller state (to track trajectories and position)
    - monitor joint sensors
HW:
    - monitor estop
    - monitor teensy heartbeat


Make sure the hw/sw always has 1 second worth of points
"""

import threading

import actionlib
import rospy

import control_msgs.msg
import control_msgs.srv
import sensor_msgs.msg
import trajectory_msgs.msg

from .. import kinematics
from . import plans


class PointSender(object):
    def __init__(self):
        self.points = []
        self._lock = threading.Lock()
        self._pid = 0
        self.gen = None
        self._position = None
        self.plan = None

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    def find_next_point(self, time=None, min_dt=0.2):
        min_dt = rospy.Duration(min_dt)
        if time is None:
            time = rospy.Time.now()
        for p in self.points:
            pdt = p.time - time
            if pdt >= min_dt:
                return p
        return None

    def find_position(self, time=None):
        if time is None:
            time = rospy.Time.now()
        dt = rospy.Duration(1000)
        ps = None
        for p in self.points:
            # look for point just before this time
            pdt = time - p.time
            if pdt > rospy.Duration(0) and pdt < dt:
                dt = pdt
                ps = p.position
        if ps is None:
            return self._position, None
        return ps, dt

    def next_pid(self):
        #if len(self.points) != 0:
        #    self._pid = max([p.pid for p in self.points])
        self._pid += 1
        return self._pid

    def drop(self, pids=None):
        if pids is None:  # if none, drop all
            self._lock.acquire()
            self.points = []
            self._lock.release()
            return
        if len(pids) == 0:
            return
        indices = []
        self._lock.acquire()
        for (i, p) in enumerate(self.points):
            if p.pid in pids:
                indices.append(i)
        for i in indices[::-1]:
            self.points.pop(i)
        self._lock.release()

    def drop_later(self, time):
        pids = []
        for p in self.points:
            if p.time > time:
                pids.append(p.pid)
        self.drop(pids)

    def drop_past(self, time=None):
        if time is None:
            time = rospy.Time.now()
        pids = []
        dt = rospy.Duration(1000)
        for p in self.points:
            if p.time < time:
                pids.append(p.pid)
                pdt = time - p.time
                if pdt > rospy.Duration(0) and pdt < dt:
                    dt = pdt
                    self._position = p.position
        self.drop(pids)

    def send_point(self, p):
        if p.pid > self._pid:
            self._pid = p.pid
        self._to_send.append(p)

    def flush(self):
        # TODO
        if len(self._to_send) == 0:
            return
        msg = control_msgs.msg.FollowJointTrajectoryGoal()
        for jn in self._joint_names:
            msg.trajectory.joint_names.append(jn)
        st = None
        to_remove = []
        now = rospy.Time.now()
        for (i, p) in enumerate(self._to_send):
            if p.time < now:
                to_remove.append(i)
            else:
                pt = trajectory_msgs.msg.JointTrajectoryPoint()
        pt1 = trajectory_msgs.msg.JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(0.)
        pt1.time_from_start = rospy.Duration(0.5)
        pt.positions = p.position
        pt1.positions = p.position
        msg.trajectory.points.append(pt)
        msg.trajectory.points.append(pt1)
        msg.trajectory.header.stamp = p.time
        self._trajectory_publisher.send_goal(msg)

    def _old_send_point(self, p):
        # TODO check limits
        if p.pid > self._pid:
            self._pid = p.pid
        self.points.append(p)
        # actually end point as trajectory
        msg = control_msgs.msg.FollowJointTrajectoryGoal()
        for jn in self._joint_names:
            msg.trajectory.joint_names.append(jn)
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        pt1 = trajectory_msgs.msg.JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(0.)
        pt1.time_from_start = rospy.Duration(0.5)
        pt.positions = p.position
        pt1.positions = p.position
        msg.trajectory.points.append(pt)
        msg.trajectory.points.append(pt1)
        msg.trajectory.header.stamp = p.time
        self._trajectory_publisher.send_goal(msg)

    def update(self, ts=None):
        # check that last sent point is at least N seconds in the future
        # if so, do nothing
        # if not, send M seconds worth of points (as a trajectory) starting
        # at the last point
        if self.gen is None:
            return
        if ts is None:
            ts = rospy.Time.now()
        self._lock.acquire()
        if len(self.points) == 0:
            self.send_point(self.gen.next())
        while self.points[-1].time - ts < rospy.Duration(1.):
            self.send_point(self.gen.next())
        self.flush()
        self._lock.release()
        self.drop_past()


class LegController(object):
    def __init__(self, name):
        self.name = name
        self.plan = None
        self._thread = None
        self._joint_names = [
            '%s_%s' % (self.name, jn) for jn in
            ('hip', 'thigh', 'knee')]
        self.points = PointSender()
        self.points._joint_names = self._joint_names
        self.connect()

    def send_foot(self, position, time):
        pass

    def connect(self):
        self._query_state = rospy.ServiceProxy(
            '/stompy/%s/query_state' % self.name,
            control_msgs.srv.QueryTrajectoryState)
        self._trajectory_publisher = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % self.name,
            control_msgs.msg.FollowJointTrajectoryAction)
        self._joint_state_sub = rospy.Subscriber(
            '/stompy/joint_states', sensor_msgs.msg.JointState,
            self._new_joint_states)
        self.points._trajectory_publisher = self._trajectory_publisher

    def _new_joint_states(self, msg):
        ps = {}
        for i in xrange(len(msg.name)):
            if msg.name[i] in self._joint_names:
                ps[msg.name[i].split('_')[1]] = msg.position[i]
        if len(ps) != 3:
            return
        foot = kinematics.frames.convert(
            [ps[j] for j in ('hip', 'thigh', 'knee')],
            kinematics.frames.JOINT_FRAME,
            kinematics.frames.BODY_FRAME)
        self.send_foot(foot, msg.header.stamp)

    def halt(self, severity):
        # TODO handle severity
        self._trajectory_publisher.cancel_goal()
        # TODO send 0 move
        self.points.drop()

    def set_plan(self, plan):
        self.previous_plan = self.plan
        self.plan = plan
        if self.plan.mode == plans.STOP_MODE:
            self._trajectory_publisher.cancel_goal()
            # TODO send 0 move
            self.points.drop()
            return
        # setup new transform
        # TODO look up start time and position by finding
        # soonest point after start_time
        if self.plan.start_time is None:
            self.plan.start_time = rospy.Time.now() + rospy.Duration(0.1)
            print("Timstamping plan: %s" % self.plan.start_time)
        self.points.gen = self.plan.point_generator(
            self.get_position(self.plan.frame, self.plan.start_time),
            self.plan.start_time,
            self.points.next_pid(), kinematics.frames.JOINT_FRAME)
        self.points.drop_later(self.plan.start_time)
        self.points.update()

    def get_position(self, frame=None, time=None):
        if frame is None:
            frame = kinematics.frames.JOINT_FRAME
        try:
            if (
                    time is None or
                    time < (rospy.Time.now() + rospy.Duration(0.05))):
                state = self._query_state(rospy.Time.now())
            else:
                state = self._query_state(time)
            joints = dict(zip(state.name, state.position))
            jpt = [joints[jn] for jn in self._joint_names]
        except rospy.ServiceException:
            print("failed to query state: %s" % time)
            ps, dt = self.points.find_position(time)
            if ps is None:
                raise NotImplementedError()
            jpt = ps
        return kinematics.frames.convert(
            jpt, kinematics.frames.JOINT_FRAME, frame)

    def update(self):
        if self.plan is None:
            return
        if self.plan.mode == plans.STOP_MODE:
            return
        self.points.update()

    def _thread_update(self):
        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(0.1)

    def run_thread(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._thread_update)
        self._thread.daemon = True
        self._thread.start()
