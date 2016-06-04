#!/usr/bin/env python

import fcntl
import os
import sys
import termios

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2
import pygazebo.msg.gz_string_pb2

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

forces = {
    'hip': 0,
    'thigh': 0,
    'knee': 0,
}

publisher = None
message = pygazebo.msg.joint_cmd_pb2.JointCmd()
subscriber = None


def callback(data):
    message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
    print('Received message:', message.data)


@trollius.coroutine
def connect(future):
    manager = yield From(
        pygazebo.connect())
    publisher = yield From(
        manager.advertise(
            '/gazebo/default/stompy/joint_cmd',
            'gazebo.msgs.JointCmd'))
    subscriber = yield From(
        manager.subscribe(
            '/gazebo/default/stompy/joint',
            'gazebo.msgs.GzString',
            callback))
    future.set_result(publisher)


def process_keys():
    global publisher
    key = sys.stdin.read(1)
    print("{} pressed".format(key))
    # u/j/m : hip
    # i/k/, : thigh
    # o/l/. : knee
    name = None
    if key == 'Q':
        loop.stop()
    elif key == 'u':
        name = 'hip'
        forces[name] += 100
    elif key == 'j':
        name = 'hip'
        forces[name] -= 100
    elif key == 'm':
        name = 'hip'
        forces[name] = 0
    elif key == 'i':
        name = 'thigh'
        forces[name] += 100
    elif key == 'k':
        name = 'thigh'
        forces[name] -= 100
    elif key == ',':
        name = 'thigh'
        forces[name] = 0
    elif key == 'o':
        name = 'knee'
        forces[name] += 100
    elif key == 'l':
        name = 'knee'
        forces[name] -= 100
    elif key == '.':
        name = 'knee'
        forces[name] = 0

    if name is not None:
        joint = joints[name]
        message.name = joint['name']
        message.axis = joint['axis']
        message.force = forces[name]
        print("%s set to %s" % (message.name, message.force))
        publisher.publish(message)

fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

print("Making event loop")
loop = trollius.get_event_loop()

print("Connecting")
future = trollius.futures.Future()
trollius.ensure_future(connect(future))
loop.run_until_complete(future)
publisher = future.result()

print("Processing keys: Q to exit")
loop.add_reader(fd, process_keys)
try:
    loop.run_forever()
except KeyboardInterrupt as e:
    pass

print("Cleaning up")
termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
