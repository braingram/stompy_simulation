#!/usr/bin/env python
"""
Inputs:
    - joystick
        - top button = deadman
        - side buttons = mode switches
        - axes = target def
    - legs
        - feet
Outputs:
    - legs
        - plans
"""

import rospy
import sensor_msgs.msg

import stompy_msgs.msg
from .. import info
from . import modes

DEADMAN_BUTTON = 1


class HeadNode(object):
    def __init__(self):
        self.mode = modes.Idle(None)
        self.connect()

    def connect(self, queue_size=10):
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.new_joystick)
        self.leg_plans = {}
        self.feet = {}
        for leg_name in info.legs:
            # pub to plan
            self.leg_plans[leg_name] = rospy.Publisher(
                '/stompy/%s/plan' % leg_name,
                stompy_msgs.msg.LegPlan, queue_size=queue_size)
            # TODO sub to foot (or do this in the mode?)
            pass

    def new_joystick(self, msg):
        # TODO throttle
        # TODO check deadman
        new_mode = self.mode.check_mode(msg)
        if new_mode is not None:  # change mode
            self.mode.exit()
            print("New mode: %s" % new_mode)
            self.mode = modes.mode_classes[new_mode](msg)
        plans = self.mode.new_input(msg)
        if plans is not None:
            self.send_plans(plans)

    def send_plans(self, plans):
        for leg_name in plans:
            print("sending to %s: %s" % (leg_name, plans[leg_name]))
            self.leg_plans[leg_name].publish(plans[leg_name])

    def update(self):
        new_mode, plans = self.mode.update()
        if new_mode is not None:
            # TODO check for new mode?
            pass
        if plans is not None:
            self.send_plans(plans)

    def run(self, dt=None):
        if dt is None:
            dt = 0.1
        while not rospy.is_shutdown():
            try:
                self.update()
                rospy.sleep(dt)
            except rospy.ROSInterruptException:
                break


def start_node(run=True):
    rospy.init_node('head')
    n = HeadNode()
    if run:
        n.run()
    else:
        return n
