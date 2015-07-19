import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2


@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    print("have manager")
    publisher = yield From(
        manager.advertise('/gazebo/default/stompy/joint_cmd',
                          'gazebo.msgs.JointCmd'))
    print("have publisher")

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'stompy::fl_leg::thigh_to_calf_upper'
    message.axis = 1
    message.force = -1000.0

    #message.name = 'stompy::fl_leg_to_hip'
    #message.axis = 2
    #message.force = 100.0

    #message.name = 'stompy::fl_leg::hip_to_thigh'
    #message.axis = 1
    #message.force = -1000.0
    print("have message")

    while True:
        yield From(publisher.publish(message))
        print("published")
        yield From(trollius.sleep(1.0))
        print("sleeping")

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
