#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointPositionRepublisher:
    def joint_state_cb(self, msg: JointState):
        pos = [msg.position[i] for i in [0, 4, 8]]       
        self.joint_pub.publish(Float64(sum(pos) / len(pos)))

    def __init__(self, topic='/allegroHand_0/joint_states'):
        rospy.loginfo(f'setting up subscriber ({topic})')
        self.joint_state = rospy.Subscriber(topic, JointState, self.joint_state_cb)

        rospy.loginfo('creating joint republisher')
        self.joint_pub = rospy.Publisher(f'{topic}_avg', Float64, queue_size=10)

        rospy.loginfo('ready')

if __name__ == '__main__':
    rospy.init_node('joint_pos_repub')
    # rospy.loginfo('Hello, World!')
    
    JointPositionRepublisher()
    rospy.spin()