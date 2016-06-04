#!/usr/bin/env python

import math
import sys

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2


assert len(sys.argv) > 1

joints = {
    'hip': {
        'name': 'stompy::body_to_fl_leg',
        'axis': 1,
    },
    'thigh': {
        'name': 'stompy::fl_leg::hip_to_thigh',
        'axis': 1,
    },
    'knee': {
        'name': 'stompy::fl_leg::thigh_to_calf_upper',
        'axis': 1,
    },
}
joints['h'] = joints['hip']
joints['t'] = joints['thigh']
joints['k'] = joints['knee']
if len(sys.argv) > 2:
    joint = joints[sys.argv[2]]
else:
    joint = joints['hip']
n = 2


@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    print("have manager")
    publisher = yield From(
        manager.advertise('/gazebo/default/stompy/joint_cmd',
                          'gazebo.msgs.JointCmd'))
    print("have publisher")

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    #message.name = 'stompy::fl_leg::thigh_to_calf_upper'
    #message.axis = 1
    #message.force = -1000.0

    #message.name = 'stompy::body_to_fl_leg'
    #message.axis = 2
    message.name = joint['name']
    message.axis = joint['axis']
    message.force = float(sys.argv[1])
    #message.force = -1000

    #message.position.p_gain = 10.0
    #message.position.i_gain = 0.5
    #message.position.target = 1.5707
    #message.position.target = math.radians(float(sys.argv[1]))

    print("message: %s" % message)
    #message.name = 'stompy::fl_leg::hip_to_thigh'
    #message.axis = 1
    #message.force = -1000.0

    for i in xrange(n):
        print("Publish: %s" % i)
        yield From(publisher.publish(message))
        yield From(trollius.sleep(1.0))

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
