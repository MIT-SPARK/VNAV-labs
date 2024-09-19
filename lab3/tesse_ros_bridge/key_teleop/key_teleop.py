#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math
import time

import rospy2 as rospy
from geometry_msgs.msg import Twist
#from ackermann_msgs.msg import AckermannDriveStamped

class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value

class TextWindow(object):

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            #raise ValueError, 'lineno out of bounds'
            raise ValueError
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = int(10)
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()



class SimpleKeyTeleop(object):
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('drive', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._backward_rate = rospy.get_param('~backward_rate', 0.5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._strafe_rate = rospy.get_param('~strafe_rate', 1.0)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0
        self._strafe = 0

        self.latest_ts = rospy.Time(0)
        
        #self._count = 0
        #self._sent = 0

    movement_bindings = {
        ord('w'): ( 1,  0,  0),
        ord('s'): (-1,  0,  0),
        ord('d'): ( 0,  1,  0),
        ord('a'): ( 0, -1,  0),
        ord('q'): ( 0,  0, -1),
        ord('e'): ( 0,  0,  1),
    }

    def run(self):
        self._running = True
        while self._running:
            t = time.time()
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            dt = time.time() - t
            time.sleep((1.0 / self._hz) - dt)

    def _get_twist(self, linear, strafe, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = strafe
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 1.0:
                keys.append(a)
        linear = 0.0
        strafe = 0.0
        angular = 0.0
        for k in keys:
            l, a, s = self.movement_bindings[k]
            linear += l
            strafe += s
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        strafe = strafe * self._strafe_rate
        angular = angular * self._rotation_rate

        self._linear = linear
        self._strafe = strafe
        self._angular = angular

    def _key_pressed(self, keycode):
        if keycode == 27:
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        twist = self._get_twist(self._linear, self._strafe, self._angular)
        ts = rospy.Time.now()
        curr = ts.sec + ts.nanosec / 10e9
        prev = self.latest_ts.sec + self.latest_ts.nanosec / 10e9

        self._interface.clear()
        self._interface.write_line(1, 'Simple Key Teleop')
        self._interface.write_line(2, 'Linear: (%f, %f), Angular: %f' % (self._linear, self._strafe, self._angular))
        self._interface.write_line(5, 'Use WASD keys to move, esc to exit.')
        #self._interface.write_line(6, f'Count: {self._count}')
        #self._interface.write_line(7, f'Duration: {self._count/self._hz:.2f}s')
        #self._interface.write_line(8, f'Msgs Sent: {self._sent}')
        #self._interface.write_line(9, f'Since pub: {curr - prev:.3f}s')
        self._interface.refresh()

        #self._count += 1

        if curr > prev:
            self._pub_cmd.publish(twist)
            self.latest_ts = ts
            #self._sent += 1



def begin(stdscr):
    rospy.init_node('key_teleop')
    #use_ack = rospy.get_param("~use_ackermann", False)

    app = None
    #if use_ack:
    #    app = SimpleKeyTeleopAckermann(TextWindow(stdscr))
    #else:
    #    app = SimpleKeyTeleop(TextWindow(stdscr))
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

def main():
    try:
        curses.wrapper(begin)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
