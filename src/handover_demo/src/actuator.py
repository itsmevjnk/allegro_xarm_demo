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
    IDLE_HANDOVER = 1
    HOME = 2
    RELEASE = 3

    PICKUP_START = 10
    PICKUP_END = 19
    HANDOVER_START = 20
    HANDOVER_END = 29

SPIN_RATE = 1000 # spinner thread's rate (in Hz)

class HandoverActuator:
    def __init__(self, ee: 'str'):
        rospy.loginfo(f'loading poses for end effector {ee}')
        with open(rospkg.RosPack().get_path('handover_demo') + '/config.yaml', 'r') as f:
            self.config = yaml.load(f)[ee]
            self.poses = self.config['arm']

        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/set_blocking')
        rospy.wait_for_service('/arm/move_joint')
        rospy.wait_for_service('/arm/intr_move')
        rospy.wait_for_service(self.config['cmds']['open'])
        rospy.wait_for_service(self.config['cmds']['close'])

        rospy.loginfo('creating service proxies')
        self.do_move_arm_home = rospy.ServiceProxy('/arm/home', Trigger)
        self.do_move_arm = rospy.ServiceProxy('/arm/move_joint', MoveArmJoints)
        self.do_intr_arm = rospy.ServiceProxy('/arm/intr_move', Trigger)
        self.do_open_ee = rospy.ServiceProxy(self.config['cmds']['open'], Trigger)
        self.do_close_ee = rospy.ServiceProxy(self.config['cmds']['close'], Trigger)

        self.holding_object = False
        self.do_open_ee() # reset end effector

        rospy.loginfo('creating handover mode topic')
        self.pose = list(self.poses.keys())[0]
        rospy.Subscriber('/act/mode', String, self.mode_cb)

        rospy.loginfo('subscribing to arm telemetry')
        self.arm_state = ActuatorMode.IDLE # does this need to be atomic?
        self.arm_staging_time = None # timestamp of when movement is staged for the arm (so we can detect timeout)
        rospy.Subscriber('/arm/status', ArmStatus, self.arm_stat_cb)

        rospy.loginfo('publishing end effector state topic')
        self.ee_closing = False
        self.ee_pub = rospy.Publisher('/act/ee_state', Bool)

        rospy.loginfo('disabling arm service blocking')
        rospy.ServiceProxy('/arm/set_blocking', SetBool)(False)

        rospy.loginfo('creating services')
        self.cmds = Queue()
        rospy.Service('/act/home', Trigger, self.home_cb)
        rospy.Service('/act/release', Trigger, self.release_cb)
        rospy.Service('/act/handover', Trigger, self.handover_cb)

        rospy.loginfo('setting up kill switch')
        self.kill = False
        rospy.Service('/act/kill', Trigger, self.killsw_cb)

        # run ourselves as spinner thread
        self.step = Steps.HOME # first command is to go to home pose
        self.step_first = False # set to indicate that we're first transitioning into step
        r = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            if self.kill: continue # stop processing until kill switch is released

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
                    if self.step >= Steps.PICKUP_START and self.step <= Steps.HANDOVER_END: # already doing handover
                        pass
                    elif self.holding_object: # object already in hand - run last step
                        self.step = Steps.HANDOVER_START + len(self.poses[self.pose]['handover']) - 1
                        self.step_first = True
                    else:
                        self.step = Steps.PICKUP_START
                        self.step_first = True
            
            # do our thing
            if self.step == Steps.IDLE_HOME or self.step == Steps.IDLE_HANDOVER: # do nothing
                pass
            elif self.step == Steps.HOME: # go to home pose
                self.step_home()
            elif self.step == Steps.RELEASE: # release hand and return to home
                self.step_release()
            elif self.step >= Steps.PICKUP_START and self.step <= Steps.HANDOVER_END: # handover
                self.step_handover()

            r.sleep()

    def mode_cb(self, data):
        self.pose = data.data
        rospy.loginfo(f'changed handover mode to {self.pose}')

    def step_home(self):
        if self.step_first:
            self.step_first = False
            self.move_arm(self.poses[self.pose]['retracted'] if self.holding_object else None)
            self.step = Steps.IDLE_HOME # we're pretty much done here

    # def move_ee(self, state):
    #     if state: self.do_close_ee()
    #     else: self.do_open_ee()

    def ee_open_cb(self, event):
        rospy.loginfo('end effector open, moving arm back to home')
        self.holding_object = False
        self.ee_pub.publish(Bool(False))
        self.cmds.put(InternalCmd.HOME)

    def step_release(self):
        if self.step_first:
            self.step_first = False
            self.do_open_ee()
            if self.config['delay'] > 0:
                rospy.Timer(rospy.Duration(self.config['delay']), self.ee_open_cb, True) # the callback does the transition - TODO: read joint velocity to determine if movement is finished
            else:
                self.ee_open_cb(None)
            
    def ee_closed_cb(self, event):
        rospy.loginfo('end effector closed')
        # self.hand_min_pos = self.hand_last_pos
        self.holding_object = True
        self.ee_pub.publish(Bool(True))
        self.step = Steps.HANDOVER_START
        self.step_first = True
        self.ee_closing = False

    def step_handover(self):
        if self.step >= Steps.PICKUP_START and self.step <= Steps.PICKUP_END:
            # picking up
            n = self.step - Steps.PICKUP_START # sub-step
            if self.step_first:
                rospy.loginfo(f'pickup step {n}')
                self.step_first = False
                self.move_arm(self.poses[self.pose]['pickup'][n])
            elif self.arm_state == ActuatorMode.IDLE:
                if n == len(self.poses[self.pose]['pickup']) - 1: # last step - time to do handover
                    if not self.ee_closing:
                        self.ee_closing = True
                        self.do_close_ee()
                        if self.config['delay'] > 0:
                            rospy.Timer(rospy.Duration(self.config['delay']), self.ee_closed_cb, True) # the callback does the transition
                        else:
                            self.ee_closed_cb(None)
                else:
                    self.step += 1
                    self.step_first = True
        else:
            # performing handover
            n = self.step - Steps.HANDOVER_START # sub-step
            if self.step_first:
                rospy.loginfo(f'handover step {n}')
                self.step_first = False
                self.move_arm(self.poses[self.pose]['handover'][n])
            elif self.arm_state == ActuatorMode.IDLE:
                if n == len(self.poses[self.pose]['handover']) - 1: # last step - go idle
                    self.step = Steps.IDLE_HANDOVER
                else:
                    self.step += 1
                    self.step_first = True
        
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

    def move_arm(self, joints = None):
        self.do_intr_arm() # interrupt current command (if there's any)
        self.arm_state = ActuatorMode.MOVE_STAGED
        self.arm_staging_time = rospy.Time.now().to_sec()
        if joints is None: self.do_move_arm_home()
        else: self.do_move_arm(joints)
    
    def home_cb(self, data):
        self.cmds.put(InternalCmd.HOME)
        return TriggerResponse(True, 'going home')
    
    def handover_cb(self, data):
        self.cmds.put(InternalCmd.HANDOVER)
        return TriggerResponse(True, ('picking up and ' if not self.holding_object else '') + 'handing over')

    def release_cb(self, data):
        self.cmds.put(InternalCmd.RELEASE)
        return TriggerResponse(True, 'releasing object' if self.holding_object else 'no objects being held')
    
    def killsw_cb(self, data):
        self.kill = not self.kill
        log = f'{"DISABLED" if self.kill else "ENABLED"} actuation'
        rospy.loginfo(log)
        return TriggerResponse(True, log)

if __name__ == '__main__':
    rospy.init_node('handover_actuator')
    
    HandoverActuator(
        str(rospy.get_param(f'{rospy.get_name()}/END_EFFECTOR'))
    )