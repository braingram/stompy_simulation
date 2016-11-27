#!/usr/bin/env python
"""
xyz_leg = body_to_leg_transform * xyz_body

stompy is facing x+, front is x+, middle is x0, rear is x-
left is y+, right is y-
"""

import numpy
#import pylab

from .. import transforms


leg_to_body_transforms = {
    'fl': transforms.affine_3d(
        2.2352, 0.5842, 0, 0, 0, 55, degrees=True),
    'fr': transforms.affine_3d(
        2.2352, -0.5842, 0, 0, 0, -55, degrees=True),
    'ml': transforms.affine_3d(
        0., 0.6858, 0, 0, 0, 90, degrees=True),
    'mr': transforms.affine_3d(
        0., -0.6858, 0, 0, 0, -90, degrees=True),
    'rl': transforms.affine_3d(
        -2.2352, 0.5842, 0, 0, 0, 125, degrees=True),
    'rr': transforms.affine_3d(
        -2.2352, -0.5842, 0, 0, 0, -125, degrees=True),
}

body_to_leg_transforms = {
    k: numpy.linalg.inv(leg_to_body_transforms[k]) for k in
    leg_to_body_transforms}

to_t = None
from_t = None
leg_name = None


def set_leg(name):
    global to_t, from_t, leg_name
    if leg_name is not None and name != leg_name:
        raise Exception("body kinematics cannot overwrite already set leg")
    leg_name = name
    to_t = leg_to_body_transforms[name]
    from_t = body_to_leg_transforms[name]


def leg_to_body(x, y, z):
    r = transforms.transform_3d(to_t, x, y, z)
    return r[0], r[1], r[2]


def leg_to_body_for_leg(leg, x, y, z):
    r = transforms.transform_3d(leg_to_body_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def leg_to_body_array(pts):
    return transforms.transform_3d_array(to_t, pts)


def leg_to_body_array_for_leg(leg, pts):
    return transforms.transform_3d_array(leg_to_body_transforms[leg], pts)


def body_to_leg(x, y, z):
    r = transforms.transform_3d(from_t, x, y, z)
    return r[0], r[1], r[2]


def body_to_leg_for_leg(leg, x, y, z):
    r = transforms.transform_3d(body_to_leg_transforms[leg], x, y, z)
    return r[0], r[1], r[2]


def body_to_leg_array(pts):
    return transforms.transform_3d_array(from_t, pts)


def body_to_leg_array_for_leg(leg, pts):
    return transforms.transform_3d_array(body_to_leg_transforms[leg], pts)
