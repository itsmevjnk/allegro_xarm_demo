#!/usr/bin/env python

import rospy
import rospkg

import yaml
from queue import Queue # thread-safe queue for passing commands to the spinner
from enum import Enum

from arm_controller.srv import SetBool,SetHand,MoveArmJoints
from arm_controller.msg import ArmStatus,JointPos
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty, String

# arm actuation mode
class ArmMode(Enum):
    IDLE = 0
    MOVE_STAGED = 1
    MOVING = 2

HAND_MOVE_DELAY = 0.5 # delay between issuing command and movement finishing (for hand)
ARM_STAGING_TIMEOUT = 0.4 # arm movement staging timeout (note that the report rate is 5Hz)

# internal commands
class InternalCmd(Enum):
    HOME = 0
    HANDOVER = 1

# movement steps
# home: hand in home pose, arm in home pose
# handover:
#  1. hand in ready pose, move arm to pickup pose
#  2. hand in grab pose (+wait)
#  3. move arm to handover pose
# if step 3 is interrupted:
#  R. move arm to pickup pose
class Steps(Enum):
    IDLE = 0

    HANDOVER_1 = 1
    HANDOVER_2 = 2
    HANDOVER_3 = 3
    HANDOVER_R = 4
    
    HOME = 10

SPIN_RATE = 1000 # spinner thread's rate (in Hz)

class HandoverActuator:
    def __init__(self):
        rospy.loginfo('loading config')
        with open(rospkg.RosPack().get_path('handover_demo') + '/config.yaml', 'r') as f:
            self.config = yaml.load(f)
        
        self.pose = 'side' # TODO: add more handover poses
        
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/set_blocking')
        rospy.wait_for_service('/arm/move_joint')
        rospy.wait_for_service('/arm/intr_move')

        rospy.loginfo('creating service proxies')
        self.do_move_arm = rospy.ServiceProxy('/arm/move_joint', MoveArmJoints)
        self.do_intr_arm = rospy.ServiceProxy('/arm/intr_move', Trigger)
        
        rospy.loginfo('creating publisher for direct hand commands')
        self.hand_cmd = rospy.Publisher('/allegroHand/lib_cmd', String)

        rospy.loginfo('subscribing to arm telemetry')
        self.arm_state = ArmMode.IDLE # does this need to be atomic?
        self.arm_staging_time = None # timestamp of when movement is staged for the arm (so we can detect timeout)
        rospy.Subscriber('/arm/status', ArmStatus, self.arm_stat_cb)

        rospy.loginfo('subscribing to hand telemetry')
        self.hand_min_pos = None # minimum hand position (for detecting yanking)
        self.hand_last_pos = None # last hand position
        rospy.Subscriber('/hand/joint_pos', JointPos, self.hand_pos_cb)

        rospy.loginfo('publishing yanking detection topic')
        self.yank = rospy.Publisher('/act/yank', Empty)

        rospy.loginfo('disabling arm service blocking')
        rospy.ServiceProxy('/arm/set_blocking', SetBool)(False)

        rospy.loginfo('creating services')
        self.cmds = Queue()
        rospy.Service('/act/home', Trigger, self.home_cb)
        rospy.Service('/act/handover', Trigger, self.handover_cb)

        # run ourselves as spinner thread
        self.step = Steps.HOME # first command is to go to home pose
        self.step_first = False # set to indicate that we're first transitioning into step
        r = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            if not self.cmds.empty():
                # there's a command waiting for us
                cmd = self.cmds.get()
                if cmd == InternalCmd.HOME:
                    if self.step != Steps.HANDOVER_R and self.step != Steps.HOME: # we don't want to interrupt ourselves reverting handover or repeat home commands
                        self.step = Steps.HOME if self.step != Steps.HANDOVER_3 else Steps.HANDOVER_R
                        self.step_first = True
                elif cmd == InternalCmd.HANDOVER:
                    if self.step >= Steps.HANDOVER_1 and self.step <= Steps.HANDOVER_3: # already doing handover
                        pass
                    elif self.step == Steps.HANDOVER_R: # go back to step 3
                        self.step = Steps.HANDOVER_3
                        self.step_first = True
                    else:
                        self.step = Steps.HANDOVER_1 
                        self.step_first = True
            
            # do our thing
            if self.step == Steps.IDLE: # do nothing
                pass
            elif self.step == Steps.HOME: # go to home pose
                self.step_home()
            elif self.step >= Steps.HANDOVER_1 and self.step <= Steps.HANDOVER_R: # handover
                self.step_handover()

            r.sleep()

    def step_home(self):
        if self.step_first:
            self.step_first = False
            self.hand_cmd.publish(String(self.config['home']['hand'])) # go back to home position
            rospy.Timer(rospy.Duration(HAND_MOVE_DELAY), self.hand_open_cb, True) # the callback does the transition
    
    def hand_open_cb(self, event):
        rospy.loginfo('hand open, moving arm back to home')
        self.move_arm(self.config['home']['arm'])
        self.step = Steps.IDLE # we're pretty much done here
    
    def step_handover(self):
        if self.step == Steps.HANDOVER_1:
            if self.step_first: # first firing
                self.step_first = False
                self.hand_cmd.publish(String(self.config['hand_pickup'])) # run hand into pickup position
                self.move_arm(self.config[self.pose]['pickup'])
            elif self.arm_state == ArmMode.IDLE: # movement is finished
                self.step = Steps.HANDOVER_2
                self.step_first = True
        elif self.step == Steps.HANDOVER_2:
            if self.step_first:
                self.step_first = False
                rospy.Timer(rospy.Duration(HAND_MOVE_DELAY), self.hand_closed_cb, True) # the callback does the transition
        elif self.step == Steps.HANDOVER_3:
            if self.step_first:
                self.step_first = False
                self.move_arm(self.config[self.pose]['handover'])
            elif self.arm_state == ArmMode.IDLE:
                self.step = Steps.IDLE
        elif self.step == Steps.HANDOVER_R:
            if self.step_first:
                self.step_first = False
                self.move_arm(self.config[self.pose]['pickup'])
            elif self.arm_state == ArmMode.IDLE:
                self.step = Steps.HOME

    def hand_closed_cb(self, event):
        rospy.loginfo('hand closed, enabling yank monitoring')
        self.hand_min_pos = self.hand_last_pos
        self.step = Steps.HANDOVER_3
        self.step_first = True

    def hand_pos_cb(self, data):
        pos = [data.pos[i] for i in self.config[self.pose]['yank']['joints']] # extract joints of interest
        self.hand_last_pos = sum(pos) / len(pos) # NOTE: does Python have a built-in average function?
        if self.hand_min_pos is not None:
            # hand pose watch is active
            dpos = (self.hand_last_pos - self.hand_min_pos)
            if dpos * self.config[self.pose]['yank']['threshold'] < 0: # reject movement in opposite direction
                self.hand_min_pos = self.hand_last_pos
            elif abs(dpos) > self.config[self.pose]['yank']['threshold']:
                rospy.loginfo('bottle yanking detected')
                self.yank.publish(Empty())
    
    def arm_stat_cb(self, data):
        if self.arm_state == ArmMode.MOVE_STAGED:
            staging_dur = rospy.Time.now().to_sec() - self.arm_staging_time
            if data.moving:
                rospy.loginfo(f'arm started moving after {staging_dur} sec')
                self.arm_state = ArmMode.MOVING
            elif staging_dur > ARM_STAGING_TIMEOUT:
                rospy.loginfo(f'arm movement staging timed out')
                self.arm_state = ArmMode.IDLE
        elif self.arm_state == ArmMode.MOVING and not data.moving:
            rospy.loginfo('arm movement completed')
            self.arm_state = ArmMode.IDLE

    def move_arm(self, joints):
        self.do_intr_arm() # interrupt current command (if there's any)
        self.arm_state = ArmMode.MOVE_STAGED
        self.arm_staging_time = rospy.Time.now().to_sec()
        self.do_move_arm(joints)
    
    def home_cb(self, data):
        self.cmds.put(ArmMode.CMD_HOME)
        return TriggerResponse(True, '')
    
    def handover_cb(self, data):
        self.cmds.put(ArmMode.CMD_HANDOVER)
        return TriggerResponse(True, '')

if __name__ == '__main__':
    rospy.init_node('handover_actuator')
    
    HandoverActuator()