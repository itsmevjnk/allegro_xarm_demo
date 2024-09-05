#!/usr/bin/env python

import rospy
import rospkg
import yaml
import random

from yolo_detector.msg import Event
from handover_demo.msg import MergedEvent
from std_srvs.srv import Trigger
from std_msgs.msg import Empty, Bool, String

HAND_MONITOR_DELAY = 0.5
HAND_DPOS_MAX = 0.05

class HandoverDemo:
    def __init__(self, ee: 'str'):
        rospy.loginfo('reading list of available poses')
        with open(rospkg.RosPack().get_path('handover_demo') + '/config.yaml', 'r') as f:
            self.poses: 'list[str]' = list(yaml.load(f)[ee]['arm'].keys())
        
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/act/home')
        rospy.wait_for_service('/act/handover')
        rospy.wait_for_service('/act/release')

        rospy.loginfo('setting up service proxies')
        self.spx_home = rospy.ServiceProxy('/act/home', Trigger)
        self.spx_handover = rospy.ServiceProxy('/act/handover', Trigger)
        self.spx_release = rospy.ServiceProxy('/act/release', Trigger)

        rospy.loginfo('setting up handover mode publisher')
        self.mode = None # to be set
        self.pub_mode = rospy.Publisher('/act/mode', String)
        self.next_mode()

        rospy.loginfo('subscribing to detectors')
        self.handover = False # set when it's time to handover
        self.holding_object = False
        rospy.Subscriber('/detectors', MergedEvent, self.detect_cb)
        # rospy.Subscriber('/act/yank', Empty, self.yank_cb)
        rospy.Subscriber('/act/ee_state', Bool, self.ee_state_cb)

        rospy.loginfo('moving to initial position')
        self.spx_home()
    
        rospy.spin()
    
    def next_mode(self, new = True):
        # select random mode
        if self.mode is None or not new:
            self.mode = random.choice(self.poses) # any random mode
        else:
            self.mode = random.choice([p for p in self.poses if p != self.mode])
        rospy.loginfo(f'setting handover mode to {self.mode}')
        self.pub_mode.publish(String(self.mode))
    
    # def yank_cb(self, data):
    #     rospy.loginfo('object yanking detected, releasing object and going back home')
    #     self.handover = False
    #     self.holding_object = False
    #     self.spx_release()
    #     # next_mode = random.choice(self.poses)
    #     # rospy.loginfo(f'next handover mode will be {next_mode}')
    #     # self.pub_mode.publish(String(next_mode))
    
    def detect_cb(self, data):
        if self.handover: handover = data.human.presence # if we're handing over the object, only the human needs to be present
        else: handover = data.human.presence and (data.object.presence or self.holding_object) # otherwise, both the object and the human need to be present (so we can grab the object)
        if self.handover != handover: # state update
            rospy.loginfo(f'handover state updated to {handover}')
            self.handover = handover

            if handover: # start handover
                self.next_mode()
                self.spx_handover()
                # holding_object will be set upon completion of grasping
            else: # go back home
                self.spx_home()
    
    def ee_state_cb(self, data):
        rospy.loginfo(f'end effector state updated to {data.data}')
        self.holding_object = data.data

if __name__ == '__main__':
    rospy.init_node('handover_demo')

    HandoverDemo(
        str(rospy.get_param(f'{rospy.get_name()}/END_EFFECTOR'))
    )
