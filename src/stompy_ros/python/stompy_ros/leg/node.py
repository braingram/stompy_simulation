#!/usr/bin/env python
"""
Leg plan executer
Inputs:
    - LegPlan.msg /leg/plan
    - heartbeat /heartbeat
    - estop /estop
    - load calibration [real only] (/leg/plan?)
Outputs:
    - joint states /stompy/joint_states
    - foot position (in body?) /stompy/feet
    - sensor readings [real only] /stompy/sensors/joints
    - estop /estop
    - near limit?

Main control loop:
    check heartbeat and estop
    broadcast joint states and foot position
    make sure controller has sufficient trajectory information
"""

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

from heartbeat import ClientHeart
from stompy_msgs.msg import LegPlan

from . import controller
from .. import kinematics
from . import plans


class LegNode(object):
    _foot_msg = geometry_msgs.msg.PointStamped
    _joint_sensor_msg = sensor_msgs.msg.JointState
    _joint_state_msg = sensor_msgs.msg.JointState
    _estop_msg = std_msgs.msg.Byte

    def __init__(self, name, controller):
        self.name = name
        self.controller = controller
        # TODO do this somewhere else?
        kinematics.body.set_leg(name)
        self.connect()

    def connect(self, queue_size=10):
        # heartbeat
        self.heart = ClientHeart(self.name, 'head')
        # connect to subscribers, setup callbacks
        rospy.Subscriber('/stompy/estop', std_msgs.msg.Byte, self.new_estop)
        rospy.Subscriber('/stompy/%s/plan' % self.name, LegPlan, self.new_plan)

        # connect to publishers
        self.publishers = {
            'foot': rospy.Publisher(
                '/stompy/%s/foot' % self.name,
                self._foot_msg,
                queue_size=queue_size),
            'joint_sensors': rospy.Publisher(
                '/stompy/%s/sensors/joints' % self.name,
                self._joint_sensor_msg,
                queue_size=queue_size),
            'joint_states': rospy.Publisher(
                '/stompy/joint_states',
                self._joint_state_msg,
                queue_size=queue_size),
            'estop': rospy.Publisher(
                '/estop',
                self._estop_msg,
                queue_size=queue_size),
        }
        self.controller.connect()
        # TODO connect to controller
        #self.controller.send_joint_states = self.send_joint_states
        #self.controller.send_joint_sensors = self.send_joint_sensors
        self.controller.send_foot = self.send_foot
        #self.controller.send_estop = self.send_estop

    # --- inputs ---
    def new_estop(self, msg):
        self.controller.halt(msg.data)

    def new_plan(self, msg):
        self.controller.set_plan(plans.from_message(msg))

    # --- outputs ---
    def send_joint_states(self, joints, time):
        """publish joint states (joint angles)

        joints : dict of key=joint name, value=angle in radians
        time: rostime of joint state measurement

        joint keys should be 'hip', 'thigh', etc
        message will prepend leg name so
        published joint names will be 'fr_hip', etc.
        """
        msg = self._joint_state_msg()
        msg.header.stamp = time
        for j in joints:
            msg.name.append('%s_%s' % (self.name, j))
            msg.position.append(joints[j])
        self.publishers['joint_states'].publish(msg)

    def send_joint_sensors(self, sensors, time):
        """publish sensor states (raw sensor readings)

        sensors : dict of key=sensor name, value=raw reading
        time : rostime of sensor readings
        """
        msg = self._joint_sensor_msg()
        msg.header.stamp = time
        for s in sensors:
            msg.name.append(s)
            msg.position.append(sensors[s])
        self.publishers['joint_sensors'].publish(msg)

    def send_foot(self, position, time):
        """publish foot position (x, y, z) in body frame"""
        msg = self._foot_msg()
        msg.point.x = position[0]
        msg.point.y = position[1]
        msg.point.z = position[2]
        msg.header.stamp = time
        self.publishers['foot'].publish(msg)

    def send_estop(self, severity):
        """publish estop with severity"""
        msg = self._estop_msg()
        msg.data = severity
        self.publishers['estop'].publish(msg)

    def update(self):
        # TODO check heart
        self.controller.update()

    def run(self, dt=None):
        if dt is None:
            dt = 0.1
        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(dt)


def start_node(leg_name, run=True):
    rospy.init_node(leg_name)
    kinematics.body.set_leg(leg_name)
    lc = controller.LegController(leg_name)
    ln = LegNode(leg_name, lc)
    if run:
        ln.run()
    else:
        return ln
