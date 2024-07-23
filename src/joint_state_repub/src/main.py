#!/usr/bin/env python

import rospy
import rosgraph

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointPositionRepublisher:
    def joint_state_cb(self, msg: JointState):
        self.joints = len(msg.position)
        for i, pub in enumerate(self.joint_pos_pub):
            pub.publish(Float64(msg.position[i]))
        for i, pub in enumerate(self.joint_vel_pub):
            pub.publish(Float64(msg.velocity[i]))

    def __init__(self, topic='/joint_state'):
        self.joints = 0 # number of joints
        self.joint_pos_pub = [] # list of joint position (re)publishers
        self.joint_vel_pub = [] # and for velocity too

        rospy.loginfo(f'setting up subscriber ({topic})')
        self.joint_state = rospy.Subscriber(topic, JointState, self.joint_state_cb)

        rospy.loginfo('waiting for first message')
        while self.joints == 0: pass

        rospy.loginfo('creating joint republishers')
        for i in range(self.joints):
            self.joint_pos_pub.append(rospy.Publisher(f'{topic}_pos/{i}', Float64, queue_size=10))
            self.joint_vel_pub.append(rospy.Publisher(f'{topic}_vel/{i}', Float64, queue_size=10))

        rospy.loginfo('ready')

if __name__ == '__main__':
    rospy.init_node('joint_state_repub')
    # rospy.loginfo('Hello, World!')
    
    JointPositionRepublisher('/allegroHand_0/joint_states')
    rospy.spin()