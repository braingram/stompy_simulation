#!/usr/bin/env python


global legs
legs = {}


def new_leg_message(name, msg):
    global legs
    legs[name] = {
        # TODO msg type with time
        'time': msg.time.to_sec(),
        'load': float(msg.load),
        'x': float(msg.position.x),
        'y': float(msg.position.y),
        'z': float(msg.position.z),
    }
