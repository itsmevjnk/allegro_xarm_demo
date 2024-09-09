#!/usr/bin/env python

import rospy
import rospkg

import yaml

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String, Bool
from arm_controller.msg import HandStatus

class YankDetector:
    def __init__(self):
        rospy.loginfo(f'loading yank detection config')
        with open(rospkg.RosPack().get_path('hand_yank_detector') + '/config.yaml', 'r') as f:
            self.config = yaml.load(f)

        rospy.loginfo('waiting for services')
        rospy.wait_for_service('/act/release')

        rospy.loginfo('creating release service proxy')
        self.release = rospy.ServiceProxy('/act/release', Trigger)

        rospy.loginfo('subscribing to handover pose')
        self.pose = None
        rospy.Subscriber('/act/mode', String, self.pose_cb)

        rospy.loginfo('subscribing to hand telemetry')
        self.min_pos = None
        self.last_pos = None
        self.detect = False # activation
        rospy.Subscriber('/hand/status', HandStatus, self.pos_cb)
        rospy.Subscriber('/act/ee_state', Bool, self.state_cb)

        rospy.spin()
    
    def pose_cb(self, data):
        self.pose = data.data
        rospy.loginfo(f'handover pose changed to {self.pose}')

    def pos_cb(self, data):
        # yank detection
        self.last_pos = data.pos
        if self.detect and self.pose is not None:
            # hand pose watch is active (only monitor once we've stopped moving and in handover position)
            for ytype in self.config['types'][self.pose]:
                cond = self.config['types'][self.pose][ytype]
                def calc_avgpos(pos: 'list[float]', joints: 'list[int]'):
                    pos_interest = [pos[i] for i in joints]
                    return sum(pos_interest) / len(pos_interest)
                dpos = calc_avgpos(self.last_pos, cond['joints']) - calc_avgpos(self.min_pos, cond['joints'])
                # rospy.loginfo(f'{ytype} hand dpos = {dpos}')
                if dpos * cond['threshold'] < 0: # reject movement in opposite direction
                    # self.min_pos = self.last_pos
                    pass
                elif abs(dpos) > abs(cond['threshold']):
                    rospy.loginfo(f'bottle yanking detected ({ytype})')
                    self.min_pos = None # detect only once
                    self.detect = False
                    self.release()
                    break
    
    def activate_detection(self, data):
        self.min_pos = self.last_pos
        self.detect = True
        rospy.loginfo(f'activated yanking detection for {self.pose}')

    def state_cb(self, data):
        if data.data: # hand closed
            rospy.Timer(rospy.Duration(self.config['activation_delay']), self.activate_detection, True)
        else:
            self.detect = False

if __name__ == '__main__':
    rospy.init_node('detector_merge')

    YankDetector()