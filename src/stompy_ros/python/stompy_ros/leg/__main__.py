#!/usr/bin/env python

import socket
import sys

from .. import info
from . import node

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args):
        leg_name = args[0]
    else:
        leg_name = socket.gethostname()
    if leg_name not in info.legs:
        raise ValueError("Invalid leg: %s" % leg_name)

    node.start_node(leg_name, run=True)
