#!/usr/bin/env python

import rospy
import threading

from yolo_detector.msg import Event
from handover_demo.msg import MergedEvent
from std_msgs.msg import Header

class DetectorMerge:
    def __init__(self):
        rospy.loginfo('publishing merged topic')
        self.pub_lock = threading.Lock() # so we don't end up with both callbacks publishing at the same time
        self.pub_seq = 0
        self.pub = rospy.Publisher('/detectors', MergedEvent, queue_size=10)

        rospy.loginfo('subscribing to human detection')
        self.human_msg = None
        rospy.Subscriber('/human_detector/events/person', Event, self.human_cb)

        rospy.loginfo('subscribing to bottle detection')
        self.bottle_msg = None
        rospy.Subscriber('/bottle_detector/events/bottle', Event, self.bottle_cb)

        rospy.spin()
    
    def publish(self):
        with self.pub_lock:
            # rospy.loginfo('publishing event')
            self.pub.publish(MergedEvent(
                Header(
                    self.pub_seq,
                    rospy.Time.now(),
                    str(self.pub_seq)
                ),
                self.human_msg,
                self.bottle_msg
            ))
            self.pub_seq += 1

    def human_cb(self, data):
        self.human_msg = data
        self.publish()
    
    def bottle_cb(self, data):
        self.bottle_msg = data
        self.publish()

if __name__ == '__main__':
    rospy.init_node('detector_merge')

    DetectorMerge()