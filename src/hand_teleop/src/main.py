#!/usr/bin/env python

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# getch implementation
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()

BINDINGS = {
    '1': ([0], 0.1),
    'q': ([0], -0.1),
    '2': ([1], 0.1),
    'w': ([1], -0.1),
    '3': ([2], 0.1),
    'e': ([2], -0.1),
    '4': ([3], 0.1),
    'r': ([3], -0.1),
    '5': ([4], 0.1),
    't': ([4], -0.1),
    '6': ([5], 0.1),
    'y': ([5], -0.1),
    '7': ([6], 0.1),
    'u': ([6], -0.1),
    '8': ([7], 0.1),
    'i': ([7], -0.1),
    'a': ([8], 0.1),
    'z': ([8], -0.1),
    's': ([9], 0.1),
    'x': ([9], -0.1),
    'd': ([10], 0.1),
    'c': ([10], -0.1),
    'f': ([11], 0.1),
    'v': ([11], -0.1),
    'g': ([12], 0.1),
    'b': ([12], -0.1),
    'h': ([13], 0.1),
    'n': ([13], -0.1),
    'j': ([14], 0.1),
    'm': ([14], -0.1),
    'k': ([15], 0.1),
    ',': ([15], -0.1),
    '9': ([1, 5, 9, 14], 0.1),
    'o': ([1, 5, 9, 14], -0.1),
    '(': ([1, 5, 9], 0.1),
    'O': ([1, 5, 9], -0.1),
    '0': ([2, 6, 10], 0.1),
    'p': ([2, 6, 10], -0.1),
    '-': ([3, 7, 11, 15], 0.1),
    '[': ([3, 7, 11, 15], -0.1),
    '_': ([3, 7, 11], 0.1),
    '{': ([3, 7, 11], -0.1)
    
}

class HandTeleop:
    def __init__(self):
        rospy.loginfo('subscribing to hand telemetry')
        self.hand_pos = None
        self.hand_update = True
        rospy.Subscriber('/allegroHand_0/joint_states', JointState, self.hand_cb)

        rospy.loginfo('publishing command topic')
        self.hand_pub = rospy.Publisher('/allegroHand_0/joint_cmd', JointState, queue_size=10)

        rospy.loginfo('waiting for first telemetry message')
        while self.hand_pos is None: pass       
        self.hand_update = False

        rospy.loginfo(f'press Space to read hand position, / to toggle continuous updating (default: {self.hand_update})')
        rospy.loginfo(f'press \' to set hand position')
        rospy.loginfo(f'press \\ to quit')

        while not rospy.is_shutdown():
            c = getch()
            if c == ' ':
                rospy.loginfo('position: ' + ','.join([str(x) for x in self.hand_pos]))
            elif c == '/':
                self.hand_update = not self.hand_update
                rospy.loginfo(f'continuous update set to {self.hand_update}')
            elif c == '\'':
                newpos = input('enter your desired joint position (or enter nothing to set to current):')
                if len(newpos) > 0: self.hand_pos = [float(pos) for pos in newpos.split(',')]
                self.hand_pub.publish(JointState(
                    Header(0, rospy.Time.now(), 'nul'),
                    '',
                    self.hand_pos, # send position only
                    [], [] # disregard velocity and effort
                ))
                rospy.loginfo(f'set hand position to ' + ','.join([str(x) for x in self.hand_pos]))
            elif c == '\\': break
            elif c in BINDINGS:
                joints, delta = BINDINGS[c]
                if self.hand_update: pos = list(self.hand_pos) # copy out for atomicity
                else: pos = self.hand_pos # by reference
                for joint in joints:
                    pos[joint] += delta
                rospy.loginfo(f'joint {joints} -> {[pos[joint] for joint in joints]}')
                self.hand_pub.publish(JointState(
                    Header(0, rospy.Time.now(), 'nul'),
                    '',
                    pos, # send position only
                    [], [] # disregard velocity and effort
                ))
    
    def hand_cb(self, data: JointState):
        if self.hand_update: self.hand_pos = list(data.position)

if __name__ == '__main__':
    rospy.init_node('hand_teleop')

    HandTeleop()
    # rospy.spin()