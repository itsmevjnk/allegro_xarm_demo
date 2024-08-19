#!/usr/bin/env python

import rospy
import rosgraph
import rospkg

import yaml

from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from arm_controller.msg import *

HAND_VEL_THRESHOLD = 100 # threshold for considering hand as moving

class AllegroHandController:
    def set_pose(self, pose):
        rospy.loginfo(f'AllegroHand: setting pose to {pose}')
        self.joint_cmd.publish(JointState(
            Header(0, rospy.Time.now(), 'nul'),
            '',
            self.poses[pose], # send position only
            [], [] # disregard velocity and effort
        ))
        return TriggerResponse(True, '')
    
    def joint_states_cb(self, data):
        # determine if hand is moving
        absvel = [abs(x) for x in data.velocity]
        moving = sum(absvel) / len(absvel) > HAND_VEL_THRESHOLD
        self.joint_pub.publish(HandStatus(
            data.header,
            data.position,
            data.velocity,
            moving
        ))

    def __init__(self, wait_rate = 10):
        rospy.loginfo('AllegroHand: reading poses from config file')
        with open(rospkg.RosPack().get_path('arm_controller') + '/poses.yaml', 'r') as f:
            self.poses = yaml.load(f)
        
        rospy.loginfo('AllegroHand: setting up publisher')
        self.joint_cmd = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=10)
        
        # courtesy of pallgeuer
        rospy.loginfo('AllegroHand: waiting for connection to topic')
        rate = rospy.Rate(wait_rate)
        master = rosgraph.Master(caller_id=rospy.get_name())
        while self.joint_cmd.get_num_connections() != sum(len(sub[1]) for sub in master.getSystemState()[1] if sub[0] == self.joint_cmd.resolved_name) and not rospy.is_shutdown():
            rate.sleep()
        
        rospy.loginfo('AllegroHand: resetting pose')
        self.set_pose('home')

        rospy.loginfo('AllegroHand: creating services')
        for p in self.poses:
            rospy.Service(
                f'/hand/{p}',
                Trigger,
                lambda msg, pose=p: self.set_pose(pose)
            )

        rospy.loginfo('AllegroHand: setting up telemetry')
        self.joint_pub = rospy.Publisher('/hand/status', HandStatus, queue_size=10)
        self.joint_states = rospy.Subscriber('/allegroHand/joint_states', JointState, self.joint_states_cb)

        rospy.loginfo('AllegroHand: ready')

if __name__ == '__main__':
    rospy.init_node('hand_controller')
    # rospy.loginfo('Hello, World!')
    
    AllegroHandController()
    rospy.spin()