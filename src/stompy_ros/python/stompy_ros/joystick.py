#!/usr/bin/env python
"""
header:
  seq: 64
  stamp:
    secs: 1495390363
    nsecs: 829546928
  frame_id: ''
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
"""


class Throttler(object):
    """
    Check that joystick has undergone at least some min change
    or has moved to 0.0 (or has button pressed?)
    """
    def __init__(self, min_moves=None):
        self.last = None
        if min_moves is None:
            min_moves = {0: 0.01, 1: 0.01, 2: 0.01}
        self.min_moves = min_moves

    def update(self, msg):
        """If msg is 'new' return True"""
        l = self.last
        self.last = msg
        if l is None:
            return True
        for bi in xrange(len(msg.buttons)):
            if l.buttons[bi] != msg.buttons[bi]:
                return True
        for ai in xrange(len(msg.axes)):
            if l.axes[ai] != 0 and msg.axes[ai] == 0:
                return True
            if abs(l.axes[ai] - msg.axes[ai]) > self.min_moves.get(ai, 0.):
                return True
        return False
