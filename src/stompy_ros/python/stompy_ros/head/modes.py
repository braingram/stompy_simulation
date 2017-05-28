#!/usr/bin/env python
"""
Control modes
modes:
     0: fl, angles, relative
     1: ml
     2: rl
     3: rr
     4: mr
     5: fr
     6: fl, foot, relative
     7: ml
     8: rl
     9: rr
     10: mr
     11: fr

     32: body translation
     33: body rotations
     34: position legs
     35: restriction control
     ...

button-mode
    4: fl
    5: ml
    6: rl
    7: rr
    8: mr
    9: fr
    10: body translation
    11: body rotations
    12: position legs
    13: restriction
"""

import rospy

from .. import info
from .. import joystick
from .. import kinematics
from .. import leg
from . import legs
from . import restriction


# foot/leg modes
#FL_JOINT = 0
#ML_JOINT = 1
#RL_JOINT = 2
#RR_JOINT = 3
#MR_JOINT = 4
#FR_JOINT = 5
#FL_FOOT = 6
#ML_FOOT = 7
#RL_FOOT = 8
#RR_FOOT = 9
#MR_FOOT = 10
#FR_FOOT = 11

IDLE = 0
MOVE_LEG = 1

leg_buttons = {
    4: 'rl',
    5: 'ml',
    6: 'fl',
    7: 'fr',
    8: 'mr',
    9: 'rr',
}

# body modes
#BODY_TRANSLATION = 32
#BODY_ROTATION = 33
MOVE_BODY = 2

body_buttons = {
    10: 'translate',
    11: 'rotate',
}

# full modes
POSITION_LEGS = 34
RESTRICTION = 35

full_buttons = {
    12: POSITION_LEGS,
    13: RESTRICTION,
}

all_buttons = {}
all_buttons.update(leg_buttons)
all_buttons.update(body_buttons)
all_buttons.update(full_buttons)


def get_pressed_button_indices(buttons):
    return [i for (i, b) in enumerate(buttons) if b]


def get_mode_from_buttons(buttons):
    if not any(buttons):
        return None
    inds = get_pressed_button_indices(buttons)
    modes = []
    for i in inds:
        if i in leg_buttons:
            modes.append(MOVE_LEG)
        elif i in body_buttons:
            #modes.append(body_buttons[i])
            modes.append(MOVE_BODY)
        elif i in full_buttons:
            modes.append(full_buttons[i])
    if len(modes) == 0:
        return None
    # return the lowest mode
    return min(modes)


class Mode(object):
    mode = None

    def __init__(self, msg=None):
        pass

    def exit(self):
        pass

    def update(self):
        new_mode = None
        plans = None  # {}
        return new_mode, plans

    def check_mode(self, msg):
        new_mode = get_mode_from_buttons(msg.buttons)
        if new_mode is None:  # no mode was requested
            return None
        if new_mode == self.mode:  # mode requested was the same
            return None
        return new_mode  # new mode requested

    def new_input(self, msg):
        plans = {}
        return plans


class Idle(Mode):
    mode = IDLE


class MoveLeg(Mode):
    mode = MOVE_LEG

    def __init__(self, msg=None):
        # TODO get move parameters from server
        self.jthrottle = joystick.Throttler()
        if msg is not None:
            self.jthrottle.update(msg)
        self.scale_angles = 1 / 100.
        self.scale_legs = 1 / 200.
        self.scale_body = 1 / 200.
        if msg is None:
            self.leg = 'fr'
            self.frame = 'body'
        else:
            inds = get_pressed_button_indices(msg.buttons)
            self.leg = None
            for i in inds:
                if i in leg_buttons:
                    self.leg = leg_buttons[i]
            if self.leg is None:
                raise ValueError(
                    "Invalid mode transition, unknown leg: %s" %
                    (msg.buttons,))
            self.frame = 'body'

    def new_input(self, msg):
        jnew = self.jthrottle.update(msg)
        # check if changing leg or angles
        inds = get_pressed_button_indices(msg.buttons)
        new_leg = None
        # TODO check for press, then release
        for i in inds:
            if i in leg_buttons:
                new_leg = leg_buttons[i]
        plans = {}
        if new_leg is not None:
            plans[self.leg] = leg.plans.make_stop_message(frame=self.frame)
            if new_leg == self.leg:
                if self.frame == 'body':
                    self.frame = 'leg'
                elif self.frame == 'leg':
                    self.frame = 'joint'
                elif self.frame == 'joint':
                    self.frame = 'body'
            else:
                self.frame = 'body'
                #plans[self.leg] = leg.plans.make_stop_message()
                self.leg = new_leg
            # TODO reprocess this input next update or just ignore it?
            return plans
        # compute new plan
        # check if axes '0'
        # compute velocity from axes
        if self.frame == 'body':
            frame = kinematics.frames.BODY_FRAME
            target = [
                msg.axes[0] * self.scale_body,
                msg.axes[1] * self.scale_body,
                msg.axes[2] * self.scale_body]
        elif self.frame == 'leg':
            frame = kinematics.frames.LEG_FRAME
            target = [
                msg.axes[0] * self.scale_legs,
                msg.axes[1] * self.scale_legs,
                msg.axes[2] * self.scale_legs]
        elif self.frame == 'joint':
            frame = kinematics.frames.JOINT_FRAME
            target = [
                msg.axes[0] * self.scale_angles,
                msg.axes[1] * self.scale_angles,
                msg.axes[2] * self.scale_angles]
        if jnew:
            plans[self.leg] = leg.plans.make_message(
                leg.plans.VELOCITY_MODE, frame, target)
        return plans


class MoveBody(Mode):
    mode = MOVE_BODY

    def __init__(self, msg=None):
        self.jthrottle = joystick.Throttler()
        if msg is not None:
            self.jthrottle.update(msg)
        # TODO get move parameters from server
        self.scale_translate = 1 / 500.
        self.scale_rotate = 1 / 500.
        if msg is None:
            self.translate = True
        else:
            inds = get_pressed_button_indices(msg.buttons)
            self.translate = None
            for i in inds:
                if i in body_buttons:
                    if body_buttons[i] == 'translate':
                        self.translate = True
                    else:
                        self.translate = False
            if self.translate is None:
                raise ValueError(
                    "Invalid mode transition, unknown body: %s" %
                    (msg.buttons,))

    def new_input(self, msg):
        jnew = self.jthrottle.update(msg)
        inds = get_pressed_button_indices(msg.buttons)
        for i in inds:
            if i in body_buttons:
                if body_buttons[i] == 'translate':
                    self.translate = True
                else:
                    self.translate = False
        if self.translate:
            plan_mode = leg.plans.VELOCITY_MODE
            target = [
                msg.axes[0] * self.scale_translate,
                msg.axes[1] * self.scale_translate,
                msg.axes[2] * self.scale_translate]
        else:
            plan_mode = leg.plans.ARC_MODE
            target = [
                0., 0., 0.,
                msg.axes[0] * self.scale_rotate,
                msg.axes[1] * self.scale_rotate,
                msg.axes[2] * self.scale_rotate]
        plans = {}
        if jnew:
            for leg_name in info.legs:
                plans[leg_name] = leg.plans.make_message(
                    plan_mode, kinematics.frames.BODY_FRAME,
                    target)
        return plans


class PositionLegs(Mode):
    mode = POSITION_LEGS

    def __init__(self, msg=None):
        # self.z_lift = -0.6
        # self.z_lower = -1.5
        # get all initial foot positions and loads
        # TODO determine support triangle
        # queue up legs to move
        self.leg = None
        self.state = None
        self.target = None
        self.last = None
        self.speed = 0.002
        self.moved_legs = []

    def exit(self):
        pass

    def check_mode(self, msg):
        new_mode = None
        return new_mode

    def update(self):
        # TODO lift legs, move to centers, place down
        if len(self.moved_legs) == len(legs.legs):
            return IDLE, {}
        new_mode = None
        plans = {}
        if self.leg is None:
            # find new leg to move
            loads = {}
            for leg_name in legs.legs:
                if leg_name in self.moved_legs:
                    continue
                loads[leg_name] = legs.legs[leg_name]['load']
            # find least loaded
            self.leg = min(loads, key=lambda l: loads[l])
            leg_state = legs.legs[self.leg]
            if leg_state['load'] > 20:
                # lift
                self.state = 'lift'
                self.target = None
                plans[self.leg] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE, kinematics.frames.BODY_FRAME,
                    (0., 0., self.speed))
            else:
                # skip lift
                self.state = 'center'
                self.target = info.foot_centers[self.leg]
                dx = self.target[0] - leg_state['x']
                dy = self.target[1] - leg_state['y']
                d = (dx * dx + dy * dy) ** 0.5
                self.last = (leg_state['time'], d)
                dx = dx / d * self.speed
                dy = dy / d * self.speed
                plans[self.leg] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE, kinematics.frames.BODY_FRAME,
                    (dx, dy, 0.0))
        else:
            # continue moving leg
            leg_state = legs.legs[self.leg]
            if self.state == 'lower':
                print('lower', self.leg, leg_state['load'])
                if leg_state['load'] > 40:
                    plans[self.leg] = leg.plans.make_stop_message(
                        frame=kinematics.frames.BODY_FRAME)
                    self.moved_legs.append(self.leg)
                    self.leg = None
            elif self.state == 'center':
                if self.last is None or leg_state['time'] != self.last[0]:
                    dx = self.target[0] - leg_state['x']
                    dy = self.target[1] - leg_state['y']
                    d = (dx * dx + dy * dy) ** 0.5
                    if self.last is not None:
                        dd = self.last[1] - d
                    else:
                        dd = 0.
                    print(
                        'center', self.leg, self.target, leg_state,
                        self.last, d, dd)
                    self.last = (leg_state['time'], d)
                    if d < 0.05 or dd < -0.001:
                        self.state = 'lower'
                        self.last = leg_state.copy()
                        plans[self.leg] = leg.plans.make_message(
                            leg.plans.VELOCITY_MODE,
                            kinematics.frames.BODY_FRAME,
                            (0., 0., -self.speed))
            elif self.state == 'lift':
                print('lower', self.leg, leg_state['load'])
                if self.target is None:
                    if leg_state['load'] < 10:
                        # continue lifting for n seconds
                        self.target = leg_state['time'] + 1
                else:
                    if leg_state['time'] > self.target:
                        self.state = 'center'
                        self.target = info.foot_centers[self.leg]
                        dx = self.target[0] - leg_state['x']
                        dy = self.target[1] - leg_state['y']
                        d = (dx * dx + dy * dy) ** 0.5
                        dx = dx / d * self.speed
                        dy = dy / d * self.speed
                        self.last = (leg_state['time'], d)
                        plans[self.leg] = leg.plans.make_message(
                            leg.plans.VELOCITY_MODE,
                            kinematics.frames.BODY_FRAME,
                            (dx, dy, 0.0))
        return new_mode, plans

    def new_input(self, msg):
        pass


class Restriction(Mode):
    mode = RESTRICTION

    def __init__(self, msg=None):
        self.jthrottle = joystick.Throttler()
        if msg is not None:
            self.jthrottle.update(msg)
        self.rc = restriction.RestrictionControl()
        self.step_size = 0.5
        self.half_step_size = self.step_size / 2.
        self.lift_velocity = 0.003
        self.lower_velocity = 0.002
        self.swing_velocity = 0.003
        self.rc.compute_foot_restrictions(self.get_foot_positions())

        for foot_name in self.rc.feet:
            self.rc.feet[foot_name].stance_target = (0, 0)
            self.rc.feet[foot_name].swing_target = \
                self.rc.feet[foot_name].center
            self.rc.feet[foot_name].swing_distance = None

        #if msg is not None:
        #    # TODO process initial joystick message
        #    self.update_targets(msg)

    def update_targets(self, msg):
        tx = msg.axes[0] / 1000.
        ty = msg.axes[1] / 1000.
        for foot_name in self.rc.feet:
            foot = self.rc.feet[foot_name]
            foot.stance_target = (-tx, -ty)
            cx, cy = foot.center
            tl = ((tx * tx) + (ty * ty)) ** 0.5
            if tl == 0.:
                foot.swing_target = (cx, cy)
            else:
                ntx, nty = tx / tl, ty / tl
                foot.swing_target = (
                    cx + ntx * self.half_step_size,
                    cy + nty * self.half_step_size)
        return self.update_plans()

    def update_plans(self, feet=None):
        if feet is None:
            feet = self.rc.feet
        plans = {}
        for foot_name in feet:
            foot = self.rc.feet[foot_name]
            # send plans
            if foot.state in ('stance', 'wait'):
                # TODO arc
                dx, dy = foot.stance_target
                plans[foot_name] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE,
                    kinematics.frames.BODY_FRAME,
                    (dx, dy, 0.))
            elif foot.state == 'lift':
                dx, dy = foot.stance_target
                plans[foot_name] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE,
                    kinematics.frames.BODY_FRAME,
                    (dx, dy, self.lift_velocity))
                foot.swing_distance = None
            elif foot.state == 'lower':
                dx, dy = foot.stance_target
                plans[foot_name] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE,
                    kinematics.frames.BODY_FRAME,
                    (dx, dy, -self.lower_velocity))
            elif foot.state == 'swing':
                leg_state = legs.legs[foot_name]
                x = leg_state['x']
                y = leg_state['y']
                tx, ty = foot.swing_target
                dx = tx - x
                dy = ty - y
                d = (dx * dx + dy * dy) ** 0.5
                sx = dx / d * self.swing_velocity
                sy = dy / d * self.swing_velocity
                if foot.swing_distance is None:
                    foot.swing_distance = d
                else:
                    foot.swing_distance = min(d, foot.swing_distance)
                plans[foot_name] = leg.plans.make_message(
                    leg.plans.VELOCITY_MODE,
                    kinematics.frames.BODY_FRAME,
                    (sx, sy, 0.))
        return plans

    def get_foot_positions(self):
        fps = {}
        for leg_name in legs.legs:
            foot = legs.legs[leg_name]
            fps[leg_name] = [foot['x'], foot['y']]
        return fps

    def update(self):
        print("Time: %s" % rospy.Time.now().to_sec())
        new_mode = None
        plans = {}
        requested_states = self.rc.update(
            rospy.Time.now().to_sec(), self.get_foot_positions())
        update_feet = []
        for foot_name in self.rc.feet:
            foot = self.rc.feet[foot_name]
            requested = requested_states.get(foot_name, None)
            leg_state = legs.legs[foot_name]
            # TODO break this out to avoid double processing
            if requested == 'pause':
                print("%s pause" % foot_name)
                foot.stance_target = (0., 0.)
                if foot.state in ('wait', 'stance'):
                    print("%s stop" % foot_name)
                    plans[foot_name] = leg.plans.make_stop_message(
                        frame=kinematics.frames.BODY_FRAME)
                    continue
                elif foot.state == 'lift':
                    print("%s lift up" % foot_name)
                    plans[foot_name] = leg.plans.make_message(
                        leg.plans.VELOCITY_MODE,
                        kinematics.frames.BODY_FRAME,
                        (0., 0., self.lift_velocity))
                elif foot.state == 'lower':
                    print("%s lower down" % foot_name)
                    plans[foot_name] = leg.plans.make_message(
                        leg.plans.VELOCITY_MODE,
                        kinematics.frames.BODY_FRAME,
                        (0., 0., -self.lower_velocity))
            if leg_state['load'] > 50:
                # leg is loaded
                if foot.state == 'lower' and leg_state['z'] <= -1.1:
                    dr = foot.restriction - foot.last_restriction
                    if dr > 0:
                        foot.set_state('stance')
                    else:
                        foot.set_state('wait')
                    print("%s to %s" % (foot_name, foot.state))
                    update_feet.append(foot_name)
            else:
                if foot.state == 'lift' and leg_state['z'] >= -0.9:
                    print("%s to swing" % foot_name)
                    foot.set_state('swing')
                    update_feet.append(foot_name)
            if requested == 'swing':
                print("%s to lift" % foot_name)
                foot.set_state('lift')
                foot.last_lift_time = rospy.Time.now().to_sec()
                # send lift
                update_feet.append(foot_name)
            if foot.state == 'swing':
                x = leg_state['x']
                y = leg_state['y']
                tx, ty = foot.swing_target
                d = ((x - tx) ** 2. + (y - ty) ** 2.) ** 0.5
                # check if near target
                if foot.swing_distance is None:
                    foot.swing_distance = d
                if d < 0.05 or (d - foot.swing_distance) > 0.05:
                    # if so, lower
                    print("%s to lower" % foot_name)
                    foot.set_state('lower')
                    update_feet.append(foot_name)
                foot.swing_distance = min(d, foot.swing_distance)
        plans.update(self.update_plans(list(set(update_feet))))
        return new_mode, plans

    def new_input(self, msg):
        jnew = self.jthrottle.update(msg)
        if jnew:
            plans = self.update_targets(msg)
        else:
            plans = {}
        return plans


mode_classes = {
    IDLE: Idle,
    MOVE_LEG: MoveLeg,
    MOVE_BODY: MoveBody,
    POSITION_LEGS: PositionLegs,
    RESTRICTION: Restriction,
}
