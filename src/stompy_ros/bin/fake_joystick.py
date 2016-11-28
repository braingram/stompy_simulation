#!/usr/bin/env python

import sys
import time

import pygame

import rospy
import sensor_msgs.msg

pygame.init()

w, h = (320, 300)
zbar = 20
delay = 100  # ms

X = 0  # ros: + left, - right
Y = 1  # ros: + forward, - back
Z = 2  # ros: + ccw, - cw
buttons = [
    pygame.K_1,  # trigger
    pygame.K_2,  # back thumb
    pygame.K_3,  # left thumb
    pygame.K_4,  # right thumb

    pygame.K_q,  # left buttons
    pygame.K_w,
    pygame.K_e,
    pygame.K_d,
    pygame.K_s,
    pygame.K_a,

    pygame.K_y,  # right buttons
    pygame.K_t,
    pygame.K_r,
    pygame.K_f,
    pygame.K_g,
    pygame.K_h,
]

button_xys = []
x = 15
y = h - 15
for i in xrange(4):
    button_xys.append((x, y))
    x += 10
y = h - 25
x = 65
for i in xrange(3):
    button_xys.append((x, y))
    x += 10
y = h - 15
x = 85
for i in xrange(3):
    button_xys.append((x, y))
    x -= 10
y = h - 25
x = 125
for i in xrange(3):
    button_xys.append((x, y))
    x -= 10
y = h - 15
x = 105
for i in xrange(3):
    button_xys.append((x, y))
    x += 10

# connect to publisher
rospy.init_node('fake_joystick', anonymous=True)
publisher = rospy.Publisher('/joy', sensor_msgs.msg.Joy, queue_size=10)

screen = pygame.display.set_mode((w, h))

global joy_data
joy_data = {
    'axes': [0, 0, 0, 0, 0, 0],
    'buttons': [
        0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
    ]
}

previous_buttons = joy_data['buttons']

xyw = w - zbar
hh = h / 2.
hw = w / 2.
hxyw = xyw / 2.

# x: [0, xyw] -> x: [+1,-1]
# y: [0, h] -> y: [+1,-1]
# z: [0, h] -> z: [+1, -1]
to_screen_x = lambda x: int((x - 1) * -hxyw)
to_screen_y = lambda y: int((y - 1) * -hh)
to_screen_z = lambda z: int((z - 1) * -hh)
from_screen_x = lambda x: -2 * (x / float(xyw)) + 1
from_screen_y = lambda y: -2 * (y / float(h)) + 1
from_screen_z = lambda z: -2 * (z / float(h)) + 1


def publish_joystick():
    global joy_data
    msg = sensor_msgs.msg.Joy()
    for a in joy_data['axes']:
        msg.axes.append(a)
    for b in joy_data['buttons']:
        msg.buttons.append(b)
    msg.header.stamp = rospy.Time.now()
    publisher.publish(msg)


while True:
    modified = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button in (1, 3):  # left, right
                x, y = event.pos
                if x < xyw:  # x, y
                    joy_data['axes'][X] = from_screen_x(x)
                    joy_data['axes'][Y] = from_screen_y(y)
                else:  # z
                    joy_data['axes'][Z] = from_screen_z(y)
                modified = True
        elif event.type == pygame.MOUSEMOTION:
            if event.buttons[0] or event.buttons[2]:
                x, y = event.pos
                if x < xyw:  # x, y
                    joy_data['axes'][X] = from_screen_x(x)
                    joy_data['axes'][Y] = from_screen_y(y)
                else:  # z
                    joy_data['axes'][Z] = from_screen_z(y)
                modified = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # left
                joy_data['axes'][X] = 0.
                joy_data['axes'][Y] = 0.
                joy_data['axes'][Z] = 0.
                modified = True
        elif event.type == pygame.KEYDOWN:
            if event.key in buttons:
                joy_data['buttons'][buttons.index(event.key)] = 1
                modified = True
        elif event.type == pygame.KEYUP:
            if event.key in buttons:
                previous_buttons = joy_data['buttons'][:]
                joy_data['buttons'][buttons.index(event.key)] = 0
                modified = True
        else:
            #print event
            pass
    # draw screen
    screen.fill((0, 0, 0))
    x = to_screen_x(joy_data['axes'][X])
    y = to_screen_y(joy_data['axes'][Y])
    pygame.draw.circle(
        screen, (255, 255, 255),
        (x, y),
        5)
    pygame.draw.line(
        screen, (255, 255, 255),
        (xyw, 0), (xyw, h))
    z = to_screen_z(joy_data['axes'][Z])
    pygame.draw.line(
        screen, (255, 255, 255),
        (xyw, z), (w, z))
    # draw button states
    for (i, b) in enumerate(joy_data['buttons']):
        x, y = button_xys[i]
        if previous_buttons[i]:
            pygame.draw.circle(screen, (255, 0, 0), (x, y), 4)
        pygame.draw.circle(screen, (255, 255, 255), (x, y), 5, 1-b)
    # publish joystick message
    if modified:
        publish_joystick()
    pygame.display.update()
    pygame.time.delay(delay)
