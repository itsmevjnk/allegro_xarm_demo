#!/usr/bin/env python

import rospy
import threading

import yolo_detector
import apriltag_detector
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
        rospy.Subscriber('/human_detector/events/person', yolo_detector.msg.Event, self.human_cb)

        rospy.loginfo('subscribing to object detection')
        self.object_msg = None
        rospy.Subscriber('/object_detector/event', apriltag_detector.msg.Event, self.object_cb)

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
                self.object_msg
            ))
            self.pub_seq += 1

    def human_cb(self, data):
        self.human_msg = data
        self.publish()
    
    def object_cb(self, data):
        self.object_msg = data
        self.publish()

if __name__ == '__main__':
    rospy.init_node('detector_merge')

    DetectorMerge()