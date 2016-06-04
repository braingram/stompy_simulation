#!/usr/bin/env python

import logging

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_pb2
import pygazebo.msg.gz_string_pb2


l = logging.getLogger('trollius')
h = logging.StreamHandler()
h.setLevel(logging.DEBUG)
l.addHandler(h)


def topic_callback(data):
    message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
    #message = pygazebo.msg.joint_pb2.Joint.FromString(data)
    print('String: %s' % message.data)


def joint_callback(data):
    message = pygazebo.msg.joint_pb2.Joint.FromString(data)
    print('Joint: %s' % message.data)


@trollius.coroutine
def subscribe_loop():
    manager = yield From(pygazebo.connect())
    #publisher = yield From(
    #    manager.advertise('/gazebo/default/topic',
    #                      'gazebo.msgs.GzString'))
    yield From(trollius.sleep(1.0))

    manager.start()
    #manager.subscribe(
    #    '/gazebo/default/topic',
    #    'gazebo.msgs.GzString',
    #    topic_callback)
    manager.subscribe(
        '/gazebo/default/stompy/joint',
        'gazebo.msgs.Joint',
        joint_callback)

    while True:
        #yield From(publisher.publish(
        #    pygazebo.msg.gz_string_pb2.GzString(data='hi')))
        yield From(trollius.sleep(1.0))

loop = trollius.get_event_loop()
loop.run_until_complete(subscribe_loop())
