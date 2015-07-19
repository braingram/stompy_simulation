#!/usr/bin/env python

import fcntl
import os
import sys
import termios

import trollius
from trollius import From


def keybindings(ch):
    return {
    }.get(ch, lambda: None)


def process_keys():
    key = sys.stdin.read(1)
    print("{} pressed".format(key))
    if key == 'Q':
        loop.stop()

fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


loop = trollius.get_event_loop()

loop.add_reader(fd, process_keys)

loop.run_forever()

termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
