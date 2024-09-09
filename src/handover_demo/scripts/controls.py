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
GP_BTN_KILL = 'CIRCLE'
GP_BTN_RELEASE = 'TRIANGLE'
GP_BTN_EXIT = 'PS'

class HandoverDemoControls:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/act/kill')
        rospy.wait_for_service('/handover/release')

        rospy.loginfo('setting up service proxies')
        self.do_kill = rospy.ServiceProxy('/act/kill', Trigger)
        self.do_release = rospy.ServiceProxy('/handover/release', Trigger)

        rospy.loginfo('waiting for gamepad connection')
        rate = rospy.Rate(10)
        while not Gamepad.available(): rate.sleep()
        rospy.loginfo('gamepad connected')

    def kill_cb(self):
        result = self.do_kill().message
        rospy.loginfo(result)
    
    def release_cb(self):
        result = self.do_release().message
        rospy.loginfo(result)

    def run(self):
        rospy.loginfo('initialising gamepad')
        gp = GP_TYPE()
        gp.startBackgroundUpdates()
        gp.addButtonPressedHandler(GP_BTN_KILL, self.kill_cb)
        gp.addButtonPressedHandler(GP_BTN_RELEASE, self.release_cb)

        rospy.loginfo(f'controls: {GP_BTN_KILL} = toggle kill switch, {GP_BTN_RELEASE} = release end effector')

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
    