#!/usr/bin/env python

import rospy
import rosgraph

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class AllegroHandController:
            
    def home(self, req=None):
        rospy.logdebug('AllegroHand: home pose')
        self.lib_cmd.publish(String('home'))

        if req is not None: return TriggerResponse(True, '') # only return if this is called as a service

    def ready(self, req=None):
        rospy.logdebug('AllegroHand: ready pose')
        self.lib_cmd.publish(String('ready'))

        if req is not None: return TriggerResponse(True, '')

    def grasp3(self, req=None):
        rospy.logdebug('AllegroHand: grasp pose (3 finger)')
        self.lib_cmd.publish(String('grasp_3'))

        if req is not None: return TriggerResponse(True, '')

    def grasp4(self, req=None):
        rospy.logdebug('AllegroHand: grasp pose (4 finger)')
        self.lib_cmd.publish(String('grasp_4'))

        if req is not None: return TriggerResponse(True, '')
    
    def pinch_idx(self, req=None):
        rospy.logdebug('AllegroHand: index pinch pose')
        self.lib_cmd.publish(String('pinch_it'))

        if req is not None: return TriggerResponse(True, '')
    
    def pinch_mid(self, req=None):
        rospy.logdebug('AllegroHand: middle pinch pose')
        self.lib_cmd.publish(String('pinch_mt'))

        if req is not None: return TriggerResponse(True, '')
    
    def envelop(self, req=None):
        rospy.logdebug('AllegroHand: envelop pose')
        self.lib_cmd.publish(String('envelop'))

        if req is not None: return TriggerResponse(True, '')

    def __init__(self, wait_rate = 10):
        rospy.loginfo('AllegroHand: setting up publisher')
        self.lib_cmd = rospy.Publisher('/allegroHand/lib_cmd', String, queue_size=10)
        
        # courtesy of pallgeuer
        rospy.loginfo('AllegroHand: waiting for connection to topic')
        rate = rospy.Rate(wait_rate)
        master = rosgraph.Master(caller_id=rospy.get_name())
        while self.lib_cmd.get_num_connections() != sum(len(sub[1]) for sub in master.getSystemState()[1] if sub[0] == self.lib_cmd.resolved_name) and not rospy.is_shutdown():
            rate.sleep()
        
        rospy.loginfo('AllegroHand: resetting pose')
        self.ready() # ready is more grasping-friendly than home

        rospy.loginfo('AllegroHand: creating services')
        self.srv_home = rospy.Service('/hand/home', Trigger, self.home)
        self.srv_ready = rospy.Service('/hand/ready', Trigger, self.ready)
        self.srv_grasp3 = rospy.Service('/hand/grasp3', Trigger, self.grasp3)
        self.srv_grasp4 = rospy.Service('/hand/grasp4', Trigger, self.grasp4)
        self.srv_pinch_idx = rospy.Service('/hand/pinch_idx', Trigger, self.pinch_idx)
        self.srv_pinch_mid = rospy.Service('/hand/pinch_mid', Trigger, self.pinch_mid)
        self.srv_envelop = rospy.Service('/hand/envelop', Trigger, self.envelop)

        rospy.loginfo('AllegroHand: ready')

if __name__ == '__main__':
    rospy.init_node('hand_controller')
    # rospy.loginfo('Hello, World!')
    
    AllegroHandController()
    rospy.spin()