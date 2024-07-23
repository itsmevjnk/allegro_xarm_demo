#!/usr/bin/env python

import rospy

from yolo_detector.msg import Event
from handover_demo.msg import MergedEvent
from std_srvs.srv import Trigger
from std_msgs.msg import Empty, Bool

HAND_MONITOR_DELAY = 0.5
HAND_DPOS_MAX = 0.05

class HandoverDemo:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/act/home')
        rospy.wait_for_service('/act/handover')
        rospy.wait_for_service('/act/release')

        rospy.loginfo('setting up service proxies')
        self.spx_home = rospy.ServiceProxy('/act/home', Trigger)
        self.spx_handover = rospy.ServiceProxy('/act/handover', Trigger)
        self.spx_release = rospy.ServiceProxy('/act/release', Trigger)

        rospy.loginfo('subscribing to detectors')
        self.handover = False # set when it's time to handover
        self.bottle_in_hand = False
        rospy.Subscriber('/detectors', MergedEvent, self.detect_cb)
        rospy.Subscriber('/act/yank', Empty, self.yank_cb)
        rospy.Subscriber('/act/hand_state', Bool, self.hand_state_cb)

        rospy.loginfo('moving to initial position')
        self.spx_home()
    
        rospy.spin()
    
    def yank_cb(self, data):
        rospy.loginfo('bottle yanking detected, releasing bottle and going back home')
        self.handover = False
        # self.bottle_in_hand = False
        self.spx_release()
    
    def detect_cb(self, data):
        if self.handover: handover = data.human.presence # if we're handing over the bottle, only the human needs to be present
        else: handover = data.human.presence and (data.bottle.presence or self.bottle_in_hand) # otherwise, both the bottle and the human need to be present (so we can grab the bottle)
        if self.handover != handover: # state update
            rospy.loginfo(f'handover state updated to {handover}')
            self.handover = handover

            if handover: # start handover
                self.spx_handover()
                # self.bottle_in_hand = True
            else: # go back home
                self.spx_home()
    
    def hand_state_cb(self, data):
        rospy.loginfo(f'hand state updated to {data.data}')
        self.bottle_in_hand = data.data

if __name__ == '__main__':
    rospy.init_node('handover_demo')

    HandoverDemo()
