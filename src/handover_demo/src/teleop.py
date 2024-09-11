#!/usr/bin/env python

import rospy
import rospkg

from std_srvs.srv import Trigger, TriggerResponse

# gamepad library
import sys
from pathlib import Path
sys.path.insert(1, rospkg.RosPack().get_path('handover_demo') + '/scripts/Gamepad')
import Gamepad

# gamepad bindings
GP_TYPE = Gamepad.PS4
GP_BTN_HOME = 'CROSS'
GP_BTN_HANDOVER = 'CIRCLE'
GP_BTN_RELEASE = 'TRIANGLE'
GP_BTN_EXIT = 'PS'

class HandoverDemoControls:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/act/home')
        rospy.wait_for_service('/act/handover')
        rospy.wait_for_service('/act/release')

        rospy.loginfo('setting up service proxies')
        self.do_home = rospy.ServiceProxy('/act/home', Trigger)
        self.do_handover = rospy.ServiceProxy('/act/handover', Trigger)
        self.do_release = rospy.ServiceProxy('/act/release', Trigger)

        rospy.loginfo('waiting for gamepad connection')
        rate = rospy.Rate(10)
        while not Gamepad.available(): rate.sleep()
        rospy.loginfo('gamepad connected')

    def home_cb(self):
        result = self.do_home().message
        rospy.loginfo(result)
    
    def handover_cb(self):
        result = self.do_handover().message
        rospy.loginfo(result)
    
    def release_cb(self):
        result = self.do_release().message
        rospy.loginfo(result)

    def run(self):
        rospy.loginfo('initialising gamepad')
        gp = GP_TYPE()
        gp.startBackgroundUpdates()
        gp.addButtonPressedHandler(GP_BTN_HOME, self.home_cb)
        gp.addButtonPressedHandler(GP_BTN_HANDOVER, self.handover_cb)
        gp.addButtonPressedHandler(GP_BTN_RELEASE, self.release_cb)

        rospy.loginfo(f'controls: {GP_BTN_HOME} = home, {GP_BTN_HANDOVER} = handover, {GP_BTN_RELEASE} = release')
        rospy.loginfo(f'press {GP_BTN_EXIT} to exit')

        try:
            while gp.isConnected():
                if gp.isPressed(GP_BTN_EXIT):
                    rospy.loginfo('exiting')
                    break

        finally:
            gp.disconnect()

if __name__ == '__main__':
    rospy.init_node('handover_controls')

    HandoverDemoControls().run()
    