#!/usr/bin/env python

import rospy

from arm_controller.srv import *
from arm_controller.msg import *
from std_srvs.srv import *

# gamepad library
import sys
from pathlib import Path
sys.path.insert(1, f'{Path(__file__).resolve().parent}/Gamepad')
import Gamepad

# gamepad bindings
GP_TYPE = Gamepad.PS3
GP_AXIS_X = 'LEFT-X'
GP_AXIS_Y = 'LEFT-Y'
GP_AXIS_Z_INC = 'L2'
GP_AXIS_Z_DEC = 'R2'
GP_BTN_GRIP = 'TRIANGLE'
GP_BTN_EXIT = 'PS'

GP_DX_MAX = 50.0
GP_DY_MAX = 50.0
GP_DZ_MAX = 50.0

class ArmTeleop:
    def __init__(self):
        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/arm/move_tool')
        rospy.wait_for_service('/hand/ready')
        rospy.wait_for_service('/hand/envelop')

        rospy.loginfo('setting up arm control service proxies')
        self.spx_move_tool = rospy.ServiceProxy('/arm/move_tool', MoveArm)
        self.spx_hand_ready = rospy.ServiceProxy('/hand/ready', Trigger)
        self.spx_hand_envelop = rospy.ServiceProxy('/hand/envelop', Trigger)

        rospy.loginfo('waiting for gamepad connection')
        rate = rospy.Rate(10)
        while not Gamepad.available(): rate.sleep()
        rospy.loginfo('gamepad connected')

        rospy.loginfo('releasing hand')
        self.spx_hand_ready()
        self.gripping = False

    def grip_cb(self):
        self.gripping = not self.gripping

        if self.gripping:
            rospy.loginfo('gripping')
            self.spx_hand_envelop()
        else:
            rospy.loginfo('releasing')
            self.spx_hand_ready()

    def run(self):
        rospy.loginfo('initialising gamepad')
        gp = GP_TYPE()
        gp.startBackgroundUpdates()
        gp.addButtonPressedHandler(GP_BTN_GRIP, self.grip_cb)

        try:
            while gp.isConnected():
                if gp.isPressed(GP_BTN_EXIT):
                    rospy.loginfo('exiting')
                    break

                # calculate dx, dy and dz
                dx = gp.axis(GP_AXIS_X) * GP_DX_MAX
                dy = gp.axis(GP_AXIS_Y) * GP_DY_MAX # note: tool Z axis is bench Y axis
                dz_inc = gp.axis(GP_AXIS_Z_INC)
                if dz_inc < 0: dz_inc = 0
                dz_dec = gp.axis(GP_AXIS_Z_DEC)
                if dz_dec < 0: dz_dec = 0
                dz = (dz_inc - dz_dec) * GP_DZ_MAX

                if dx != 0 or dy != 0 or dz != 0:
                    rospy.loginfo(f'dx = {dx}, dy = {dy}, dz = {dz}')
                    self.spx_move_tool([dx, dz, dy], [0, 0, 0])

        finally:
            gp.disconnect()

if __name__ == '__main__':
    rospy.init_node('arm_teleop')

    ArmTeleop().run()
    