#!/usr/bin/env python

import rospy
import rospkg

import yaml
from queue import Queue # thread-safe queue for passing commands to the spinner
from enum import IntEnum
from threading import Lock

from arm_controller.srv import SetBool,MoveArmJoints
from arm_controller.msg import ArmStatus,HandStatus
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty, String, Bool

# arm actuation mode
class ActuatorMode(IntEnum):
    IDLE = 0
    MOVE_STAGED = 1
    MOVING = 2

HAND_STAGING_TIMEOUT = 0.2 # hand movement staging timeout
ARM_STAGING_TIMEOUT = 0.3 # arm movement staging timeout (note that the report rate is 5Hz)

# internal commands
class InternalCmd(IntEnum):
    HOME = 0
    HANDOVER = 1
    RELEASE = 2

# movement steps
# home: hand in home pose, arm in home pose
# handover:
# if no object in hand:
#  1. hand in ready pose, move arm to pickup pose
#  2. hand in grab pose (+wait)
# in either case:
#  3. move arm to handover pose
class Steps(IntEnum):
    IDLE_HOME = 0

    HANDOVER_1 = 10
    HANDOVER_2 = 11
    HANDOVER_3 = 12
    IDLE_HANDOVER = 13

    HOME = 20

    RELEASE = 30

SPIN_RATE = 1000 # spinner thread's rate (in Hz)

class HandoverActuator:
    def __init__(self):
        rospy.loginfo('loading config')
        with open(rospkg.RosPack().get_path('handover_demo') + '/config.yaml', 'r') as f:
            self.config = yaml.load(f)
        
        rospy.loginfo('retrieving required poses')
        poses = set()
        poses.add(self.config['home']['hand']['pose'])
        for pose in self.config['handover']:
            poses.add(self.config['handover'][pose]['hand']['pickup'])
            poses.add(self.config['handover'][pose]['hand']['grasp'])
        services = {p: f'/hand/{p}' for p in poses} # create list of services

        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/set_blocking')
        rospy.wait_for_service('/arm/move_joint')
        rospy.wait_for_service('/arm/intr_move')
        for srv in services.values(): rospy.wait_for_service(srv)

        rospy.loginfo('creating service proxies')
        self.do_move_arm = rospy.ServiceProxy('/arm/move_joint', MoveArmJoints)
        self.do_intr_arm = rospy.ServiceProxy('/arm/intr_move', Trigger)
        self.hand_cmd = {p: rospy.ServiceProxy(services[p], Trigger) for p in services}

        rospy.loginfo('creating handover mode topic')
        self.pose = 'over' # TODO: add more handover poses
        rospy.Subscriber('/act/mode', String, self.mode_cb)

        #rospy.loginfo('creating publisher for direct hand commands')
        #self.hand_cmd = rospy.Publisher('/allegroHand/lib_cmd', String)

        rospy.loginfo('subscribing to arm telemetry')
        self.arm_state = ActuatorMode.IDLE # does this need to be atomic?
        self.arm_staging_time = None # timestamp of when movement is staged for the arm (so we can detect timeout)
        rospy.Subscriber('/arm/status', ArmStatus, self.arm_stat_cb)

        rospy.loginfo('subscribing to hand telemetry')
        self.hand_min_pos = None # minimum hand position (for detecting yanking) - also used to detect if hand is grabbing anything
        self.hand_last_pos = None # last hand position
        self.hand_state = ActuatorMode.IDLE
        self.hand_staging_time = None
        rospy.Subscriber('/hand/status', HandStatus, self.hand_stat_cb)

        rospy.loginfo('publishing yanking detection topic')
        self.yank = rospy.Publisher('/act/yank', Empty)

        rospy.loginfo('publishing hand state topic')
        self.hand_pub = rospy.Publisher('/act/hand_state', Bool)

        rospy.loginfo('disabling arm service blocking')
        rospy.ServiceProxy('/arm/set_blocking', SetBool)(False)

        rospy.loginfo('creating services')
        self.cmds = Queue()
        rospy.Service('/act/home', Trigger, self.home_cb)
        rospy.Service('/act/release', Trigger, self.release_cb)
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
                    self.step = Steps.HOME
                    self.step_first = True
                elif cmd == InternalCmd.RELEASE:
                    self.step = Steps.RELEASE
                    self.step_first = True
                elif cmd == InternalCmd.HANDOVER:
                    if self.step >= Steps.HANDOVER_1 and self.step <= Steps.HANDOVER_3: # already doing handover
                        pass
                    elif self.hand_min_pos is not None: # object already in hand
                        self.step = Steps.HANDOVER_3
                        self.step_first = True
                    else:
                        self.step = Steps.HANDOVER_1 
                        self.step_first = True
            
            # do our thing
            if self.step == Steps.IDLE_HOME or self.step == Steps.IDLE_HANDOVER: # do nothing
                pass
            elif self.step == Steps.HOME: # go to home pose
                self.step_home()
            elif self.step == Steps.RELEASE: # release hand and return to home
                self.step_release()
            elif self.step >= Steps.HANDOVER_1 and self.step <= Steps.HANDOVER_3: # handover
                self.step_handover()

            r.sleep()

    def mode_cb(self, data):
        self.pose = data.data
        rospy.loginfo(f'changed handover mode to {self.pose}')

    def step_home(self):
        if self.step_first:
            self.step_first = False
            self.move_arm(self.config['home']['arm'] if self.hand_min_pos is None else self.config['handover'][self.pose]['retracted'])
            self.step = Steps.IDLE_HOME # we're pretty much done here

    def move_hand(self, cmd, wait=False):
        self.hand_state = ActuatorMode.MOVE_STAGED
        self.hand_staging_time = rospy.Time.now().to_sec()
        self.hand_cmd[cmd]()
        if wait:
            while self.hand_state != ActuatorMode.IDLE: pass

    def hand_open_cb(self, event):
        rospy.loginfo('hand open, moving arm back to home')
        self.hand_pub.publish(Bool(False))
        self.cmds.put(InternalCmd.HOME)

    def step_release(self):
        if self.step_first:
            self.step_first = False
            self.move_hand(self.config['home']['hand']['pose']) # go back to home position
            rospy.Timer(rospy.Duration(self.config['home']['hand']['delay']), self.hand_open_cb, True) # the callback does the transition - TODO: read joint velocity to determine if movement is finished
    
    def hand_closed_cb(self, event):
        rospy.loginfo('hand closed, enabling yank monitoring')
        self.hand_min_pos = self.hand_last_pos
        self.hand_pub.publish(Bool(True))
        self.step = Steps.HANDOVER_3
        self.step_first = True

    def step_handover(self):
        if self.step == Steps.HANDOVER_1:
            if self.step_first: # first firing
                self.step_first = False
                self.move_hand(self.config['handover'][self.pose]['hand']['pickup']) # run hand into pickup position
                self.move_arm(self.config['handover'][self.pose]['pickup'])
            elif self.arm_state == ActuatorMode.IDLE and self.hand_state == ActuatorMode.IDLE: # movement is finished
                self.step = Steps.HANDOVER_2
                self.step_first = True
        elif self.step == Steps.HANDOVER_2:
            if self.step_first:
                self.step_first = False
                self.move_hand(self.config['handover'][self.pose]['hand']['grasp']) # run hand into grasping position
                rospy.Timer(rospy.Duration(self.config['handover'][self.pose]['hand']['delay']), self.hand_closed_cb, True) # the callback does the transition
        elif self.step == Steps.HANDOVER_3:
            if self.step_first:
                self.step_first = False
                self.move_arm(self.config['handover'][self.pose]['handover'])
            elif self.arm_state == ActuatorMode.IDLE:
                self.hand_min_pos = self.hand_last_pos # update initial position in case the joints drift
                self.step = Steps.IDLE_HANDOVER

    def hand_stat_cb(self, data):
        # yank detection
        self.hand_last_pos = data.pos
        if self.hand_min_pos is not None and self.step == Steps.IDLE_HANDOVER:
            # hand pose watch is active (only monitor once we've stopped moving and in handover position)
            for ytype in self.config['handover'][self.pose]['yank']:
                cond = self.config['handover'][self.pose]['yank'][ytype]
                def calc_avgpos(pos: 'list[float]', joints: 'list[int]'):
                    pos_interest = [pos[i] for i in joints]
                    return sum(pos_interest) / len(pos_interest)
                dpos = calc_avgpos(self.hand_last_pos, cond['joints']) - calc_avgpos(self.hand_min_pos, cond['joints'])
                # rospy.loginfo(f'{ytype} hand dpos = {dpos}')
                if dpos * cond['threshold'] < 0: # reject movement in opposite direction
                    # self.hand_min_pos = self.hand_last_pos
                    pass
                elif abs(dpos) > abs(cond['threshold']):
                    rospy.loginfo(f'bottle yanking detected ({ytype})')
                    self.hand_min_pos = None # detect only once
                    self.yank.publish(Empty())
                    break
        
        # set hand mode
        if self.hand_state == ActuatorMode.MOVE_STAGED:
            staging_dur = rospy.Time.now().to_sec() - self.hand_staging_time
            if data.moving:
                rospy.loginfo(f'hand started moving after {staging_dur} sec')
                self.hand_state = ActuatorMode.MOVING
            elif staging_dur > HAND_STAGING_TIMEOUT:
                rospy.logwarn(f'hand movement staging timed out')
                self.hand_state = ActuatorMode.IDLE
        elif self.hand_state == ActuatorMode.MOVING and not data.moving:
            rospy.loginfo('hand movement completed')
            self.hand_state = ActuatorMode.IDLE
        
    def arm_stat_cb(self, data):
        if self.arm_state == ActuatorMode.MOVE_STAGED:
            staging_dur = rospy.Time.now().to_sec() - self.arm_staging_time
            if data.moving:
                rospy.loginfo(f'arm started moving after {staging_dur} sec')
                self.arm_state = ActuatorMode.MOVING
            elif staging_dur > ARM_STAGING_TIMEOUT:
                rospy.logwarn(f'arm movement staging timed out')
                self.arm_state = ActuatorMode.IDLE
        elif self.arm_state == ActuatorMode.MOVING and not data.moving:
            rospy.loginfo('arm movement completed')
            self.arm_state = ActuatorMode.IDLE

    def move_arm(self, joints):
        self.do_intr_arm() # interrupt current command (if there's any)
        self.arm_state = ActuatorMode.MOVE_STAGED
        self.arm_staging_time = rospy.Time.now().to_sec()
        self.do_move_arm(joints)
    
    def home_cb(self, data):
        self.cmds.put(InternalCmd.HOME)
        return TriggerResponse(True, '')
    
    def handover_cb(self, data):
        self.cmds.put(InternalCmd.HANDOVER)
        return TriggerResponse(True, '')

    def release_cb(self, data):
        self.cmds.put(InternalCmd.RELEASE)
        return TriggerResponse(True, '')

if __name__ == '__main__':
    rospy.init_node('handover_actuator')
    
    HandoverActuator()