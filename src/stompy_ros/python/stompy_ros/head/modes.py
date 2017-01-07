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

from .. import info
from .. import kinematics
from .. import leg
from . import legs


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
        plans[self.leg] = leg.plans.make_message(
            leg.plans.VELOCITY_MODE, frame, target)
        return plans


class MoveBody(Mode):
    mode = MOVE_BODY

    def __init__(self, msg=None):
        # TODO get move parameters from server
        self.scale_translate = 1 / 200.
        self.scale_rotate = 1 / 100.
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


mode_classes = {
    IDLE: Idle,
    MOVE_LEG: MoveLeg,
    MOVE_BODY: MoveBody,
    POSITION_LEGS: PositionLegs,
    RESTRICTION: Restriction,
}
