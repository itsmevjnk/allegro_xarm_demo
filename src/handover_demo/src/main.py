#!/usr/bin/env python

import rospy

from arm_controller.srv import *
from arm_controller.msg import *
from std_srvs.srv import *

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

ARM_RETRACT = [-1.6563962697982788,0.2860989272594452,-0.8479730486869812,0.0007228884496726096,3.155287265777588,1.7915362119674683,2.0547096729278564]
ARM_EXTEND = [-0.9187011122703552,0.3914373815059662,-1.4527181386947632,1.1104869842529297,1.8421498537063599,1.1605887413024902,2.9856021404266357]

HAND_MONITOR_DELAY = 0.5
HAND_DPOS_MAX = 0.05

class HandoverDemo:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/move_joint')
        rospy.wait_for_service('/arm/intr_move')
        rospy.wait_for_service('/hand/ready')
        rospy.wait_for_service('/hand/envelop')

        rospy.loginfo('setting up service proxies')
        self.spx_move_joint = rospy.ServiceProxy('/arm/move_joint', MoveArmJoints)
        self.spx_hand_release = rospy.ServiceProxy('/hand/ready', Trigger)
        self.spx_hand_grasp = rospy.ServiceProxy('/hand/envelop', Trigger)
        self.spx_interrupt_move = rospy.ServiceProxy('/arm/intr_move', Trigger)

        rospy.loginfo('subscribing to hand telemetry')
        self.hand_min_pos = None # minimum hand position (for detecting yanking)
        self.hand_last_pos = None # last hand position
        self.hand_sub = rospy.Subscriber('/hand/joint_pos', JointPos, self.hand_pos_cb)

        rospy.loginfo('disabling arm service blocking')
        rospy.ServiceProxy('/arm/set_blocking', arm_controller.srv.SetBool)(False)

        rospy.loginfo('moving to initial position')
        self.arm_retract()
        self.hand_release()

        rospy.loginfo('available commands: E = extend, R = retract, D = release bottle, G = grab bottle, Q = quit')
        while not rospy.is_shutdown():
            c = getch().lower()
            if c == 'e':
                self.arm_extend()
            elif c == 'r':
                self.arm_retract()
            elif c == 'd':
                self.hand_release()
            elif c == 'g':
                self.hand_grasp()
            elif c == 'q':
                break

    def arm_extend(self):
        rospy.loginfo('extending arm')
        self.spx_interrupt_move()
        self.spx_move_joint(ARM_EXTEND)

    def arm_retract(self):
        rospy.loginfo('retracting arm')
        self.spx_interrupt_move()
        self.spx_move_joint(ARM_RETRACT)
    
    def hand_release(self):
        rospy.loginfo('releasing bottle')
        self.hand_min_pos = None
        self.spx_hand_release()

    def hand_start_monitor(self, event=None):
        rospy.loginfo('enabling yank monitoring')
        self.hand_min_pos = self.hand_last_pos

    def hand_grasp(self):
        rospy.loginfo('grabbing bottle')
        self.spx_hand_grasp()
        rospy.Timer(rospy.Duration(HAND_MONITOR_DELAY), self.hand_start_monitor, True)

    def hand_pos_cb(self, data):
        pos = [data.pos[i] for i in [0, 4, 8]] # extract joints of interest
        self.hand_last_pos = sum(pos) / len(pos) # NOTE: does Python have a built-in average function?
        if self.hand_min_pos is not None:
            # hand pose watch is active
            if self.hand_last_pos < self.hand_min_pos:
                self.hand_min_pos = self.hand_last_pos
            elif self.hand_last_pos - self.hand_min_pos > HAND_DPOS_MAX:
                self.yank_cb()
    
    def yank_cb(self):
        rospy.loginfo('bottle yanking detected')
        self.hand_release()
        self.arm_retract()

if __name__ == '__main__':
    rospy.init_node('handover_demo')

    HandoverDemo()
    # rospy.spin()