#!/usr/bin/env python

import sys

import numpy

import rospy
import std_msgs.msg

import leg


# hips are 45 degrees (0.7853 radians) offset
# x' = x cos t - y sin t
# y' = y cos t + x sin t
hip_angles = {
    'fl': numpy.pi / 4.,
    'fr': -numpy.pi / 4.,
    'ml': numpy.pi / 2.,
    'mr': -numpy.pi / 2.,
    'rl': numpy.pi * 3 / 4.,
    'rr': -numpy.pi * 3 / 4.,
}

legs = ('fr', 'fl', 'mr', 'ml', 'rr', 'rl')
left_legs = [l for l in legs if l[1] == 'l']
right_legs = [l for l in legs if l[1] == 'r']
front_legs = [l for l in legs if l[0] == 'f']
mid_legs = [l for l in legs if l[0] == 'm']
rear_legs = [l for l in legs if l[0] == 'r']
joints = ('hip', 'thigh', 'knee')
lc = lambda leg, joint: '/stompy/%s_%s_controller/command' % (leg, joint)


def rotate_point(a, x, y, z):
    sa = numpy.sin(a)
    ca = numpy.cos(a)
    return (x * ca - y * sa, y * ca + x * sa, z)


def project_point(l, x, y, z):
    return rotate_point(hip_angles[l], x, y, z)


def unproject_point(l, x, y, z):
    return rotate_point(-hip_angles[l], x, y, z)


t = numpy.arange(3000) * 0.1
gx = (
    (t - 5) * (t > 5) * (t < 25) +
    (10-t * 2) * (t <= 5) +
    (20-(t-25) * 2) * (t >= 25))
gz = 1. / (1. + numpy.exp(2*(5.0-t))) - 1. / (1 + numpy.exp(2*(25.0-t))) - 0.5


def stand(start_z=0.2, end_z=1.2, dz=0.01, x=0.9, y=0.):
    z = start_z
    while z < end_z:
        s = {}
        for l in legs:
            px, py, pz = x, y, z
            #if l[1] == 'l':
            #    px, py, pz = unproject_point(l, x, y, z)
            #elif l[1] == 'r':
            #    px, py, pz = unproject_point(l, x, -y, z)
            s[l] = leg.compute_angles(px, py, pz)
        z += dz
        yield s


def position_leg(l, ex, ey, sx, sy, zl=1.2, zr=0.2, dz=0.01, bs=None):
    if bs is None:
        bs = {}
    z = zl
    # lift
    print("lifting")
    while z > zr:
        s = bs.copy()
        s[l] = leg.compute_angles(sx, sy, z)
        yield s
        z -= dz
    z = zr
    # move
    dx = (ex - sx) / 50.
    dy = (ey - sy) / 50.
    x = sx
    y = sy
    vx = (dx == 0) or abs(x - ex) < abs(dx)
    vy = (dy == 0) or abs(y - ey) < abs(dy)
    print("moving")
    while not vx or not vy:
        s = bs.copy()
        s[l] = leg.compute_angles(x, y, z)
        yield s
        if not vx:
            x += dx
        else:
            x = ex
        if not vy:
            y += dy
        else:
            y = ey
        vx = (dx == 0) or abs(x - ex) < abs(dx)
        vy = (dy == 0) or abs(y - ey) < abs(dy)
    # lower
    print("lowering")
    while z < zl:
        s = bs.copy()
        s[l] = leg.compute_angles(ex, ey, z)
        yield s
        z += dz


def walk(y=1.1):
    # get legs to starting positions
    # lift  3 legs
    # move high legs forward at 2x speed
    # move low legs back at 1x speed
    # lower legs
    pass


def init():
    rospy.init_node('stompy_stand', anonymous=True)


def subscribe(queue_size=10):
    ps = {}
    for l in legs:
        for j in joints:
            tn = lc(l, j)
            ps[tn] = rospy.Publisher(
                tn, std_msgs.msg.Float64, queue_size=queue_size)
    return ps


def run(function, pubs, state, *args, **kwargs):
    for s in function(*args, **kwargs):
        state.update(s)
        for l in state:
            h, t, k = state[l]
            print("%s: %s, %s, %s" % (l, h, t, k))
            pubs[lc(l, 'hip')].publish(h)
            pubs[lc(l, 'thigh')].publish(t)
            pubs[lc(l, 'knee')].publish(k)
        yield state


if __name__ == '__main__':
    init()
    ps = subscribe()
    g = None
    #def stand(start_z=0.2, end_z=1.2, dz=0.005, x=0., y=0.9):
    #def position_leg(l, ex, ey, sx, sy, zl=1.2, zr=0.2, dz=0.005, bs=None):
    sx = 0.8
    ex = 1.1
    sz = 0.1
    #mz = 0.5
    mz = 1.1
    hz = 2.0
    funcs = [
        (stand, (), {'x': sx, 'start_z': sz, 'end_z': mz}),
        (position_leg, ('fl', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (position_leg, ('fr', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (position_leg, ('ml', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (position_leg, ('mr', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (position_leg, ('rl', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (position_leg, ('rr', sx, 0., ex, 0.), {'zr': sz, 'zl': mz}),
        (stand, (), {'x': ex, 'start_z': mz, 'end_z': hz}),
    ]
    i = 0
    state = {}
    while not rospy.is_shutdown():
        if g is None:
            if i < len(funcs):
                f, a, k = funcs[i]
                print("running: %s" % f.__name__)
                g = run(f, ps, state, *a, **k)
                i += 1
        else:
            try:
                state = g.next()
            except StopIteration:
                g = None
        rospy.sleep(0.1)
