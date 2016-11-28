#!/usr/bin/env python

from . import body
from . import leg


SENSOR_FRAME = 0
JOINT_FRAME = 1
LEG_FRAME = 2
BODY_FRAME = 3

frame_names = {
    'sensor': SENSOR_FRAME,
    'joint': JOINT_FRAME,
    'leg': LEG_FRAME,
    'body': BODY_FRAME,
}
frames = frame_names.values()


def lookup_frame(frame):
    if frame in frames:
        return frame
    if frame in frame_names:
        return frame_names[frame]
    raise ValueError("Unknown frame: %s" % frame)


def convert_pts(pts, from_frame, to_frame):
    if from_frame == to_frame:
        return pts
    if from_frame not in frames:
        raise ValueError("Invalid frame: %s" % from_frame)
    if to_frame not in frames:
        raise ValueError("Invalid frame: %s" % to_frame)
    if from_frame == SENSOR_FRAME or to_frame == SENSOR_FRAME:
        return NotImplementedError("sensor frame not implemented")
    if from_frame == BODY_FRAME:
        lpts = body.body_to_leg_array(pts)
        if to_frame == LEG_FRAME:
            return lpts
        jpts = leg.leg_to_joints_array(lpts)
        return jpts
    elif from_frame == LEG_FRAME:
        if to_frame == BODY_FRAME:
            return body.leg_to_body_array(pts)
        else:  # JOINT_FRAME
            return leg.leg_to_joints_array(pts)
    else:  # JOINT_FRAME
        lpts = leg.joints_to_leg_array(pts)
        if to_frame == LEG_FRAME:
            return lpts
        bpts = body.leg_to_body_array(lpts)
        return bpts


def convert(pt, from_frame, to_frame):
    return convert_pts([pt, ], from_frame, to_frame)[0]
