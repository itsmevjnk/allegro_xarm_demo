#!/usr/bin/env python

import rospy

from arm_controller.srv import *
from arm_controller.msg import *
from std_srvs.srv import *
from yolo_detector.msg import *
from yolo_detector.srv import *

ARM_RETRACT = [-1.3946762084960938,0.8651095628738403,0.3751215934753418,1.3067330121994019,1.829132080078125,1.2459126710891724,2.1973795890808105]
ARM_EXTEND = [-2.4859461784362793,0.8843361139297485,0.16265948116779327,1.871404767036438,1.6797127723693848,1.2428677082061768,2.9693477153778076]

HAND_MONITOR_DELAY = 0.5
HAND_DPOS_MAX = 0.05

class HandoverDemo:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/move_joint')
        rospy.wait_for_service('/arm/intr_move')
        rospy.wait_for_service('/hand/ready')
        rospy.wait_for_service('/hand/envelop')
        rospy.wait_for_service('/bottle_detector/pres/bottle')
        rospy.wait_for_service('/human_detector/pres/person')

        rospy.loginfo('setting up service proxies')
        self.spx_move_joint = rospy.ServiceProxy('/arm/move_joint', MoveArmJoints)
        self.spx_hand_release = rospy.ServiceProxy('/hand/ready', Trigger)
        self.spx_hand_grasp = rospy.ServiceProxy('/hand/envelop', Trigger)
        self.spx_interrupt_move = rospy.ServiceProxy('/arm/intr_move', Trigger)
        self.spx_bottle_present = rospy.ServiceProxy('/bottle_detector/pres/bottle', GetPresence)
        self.spx_human_present = rospy.ServiceProxy('/human_detector/pres/person', GetPresence)

        self.robot_op = False # thread safety

        rospy.loginfo('subscribing to hand telemetry')
        self.hand_min_pos = None # minimum hand position (for detecting yanking)
        self.hand_last_pos = None # last hand position
        self.hand_sub = rospy.Subscriber('/hand/joint_pos', JointPos, self.hand_pos_cb)

        rospy.loginfo('subscribing to human detection')
        self.human_sub = rospy.Subscriber('/human_detector/events/person', Event, self.human_cb)

        rospy.loginfo('subscribing to bottle detection')
        self.human_sub = rospy.Subscriber('/bottle_detector/events/bottle', Event, self.bottle_cb)

        # rospy.loginfo('disabling arm service blocking')
        # rospy.ServiceProxy('/arm/set_blocking', arm_controller.srv.SetBool)(False)

        rospy.loginfo('moving to initial position')
        self.hand_release()
        self.arm_retract()
    
        rospy.spin()

    def arm_extend(self):
        rospy.loginfo('extending arm')
        # self.spx_interrupt_move()
        self.spx_move_joint(ARM_EXTEND)

    def arm_retract(self):
        rospy.loginfo('retracting arm')
        # self.spx_interrupt_move()
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
    
    def human_cb(self, data):
        if self.robot_op: return # another thread has got this
        self.robot_op = True
        if data.presence:
            rospy.loginfo('human entered view')
            if self.spx_bottle_present().presence: # only swing out if there's a bottle that we can grab
                self.hand_grasp(); rospy.sleep(0.5) # there seems to be no way to check if pose setting is complete
                self.arm_extend()
        else:
            rospy.loginfo('human exited view')
            self.arm_retract()
            self.hand_release()
        self.robot_op = False
    
    def bottle_cb(self, data):
        if data.presence:
            rospy.loginfo('bottle entered view')
            if self.robot_op: return # another thread has got this
            if self.spx_human_present().presence: # only swing out if there's a bottle that we can grab
                self.robot_op = True
                self.hand_grasp(); rospy.sleep(0.5) # there seems to be no way to check if pose setting is complete
                self.arm_extend()
                self.robot_op = False
    
        

if __name__ == '__main__':
    rospy.init_node('handover_demo')

    HandoverDemo()
