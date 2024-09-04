#!/usr/bin/env python3

import rospy
import cv2
import apriltag

from std_msgs.msg import Header
from apriltag_detector.msg import Event
from apriltag_detector.srv import GetPresence, GetPresenceResponse

class AprilTagDetector:
    def __init__(self, namespace: str, obj_name: str, cam_id: int, cam_resolution: 'str | None' = None, enter_min: float = 1.0, exit_max: float = 0.5, show_cam: bool = False):
        self.cap = cv2.VideoCapture(cam_id) # start camera
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if cam_resolution is not None and len(cam_resolution) > 0: # set custom resolution
            cam_resolution = cam_resolution.split('x')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam_resolution[0]))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam_resolution[1]))
        self.presence_raw = False
        self.presence_filtered = False
        self.presence_timestamp = 0.0

        self.detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

        # save args
        self.enter_min = enter_min
        self.exit_max = exit_max
        self.show_cam = show_cam

        # start interface
        rospy.loginfo(f'apriltag_detector: starting ROS interface (as {obj_name})')
        self.event_seq = 0
        self.pub_event = rospy.Publisher(f'{namespace}/events/{obj_name}', Event, queue_size=10) # publisher
        self.srv_presence = rospy.Service(f'{namespace}/pres/{obj_name}', GetPresence, self.srv_presence_cb)

        self.frames = 0 # frame counter
        self.run() # start running

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def srv_presence_cb(self, msg):
        return GetPresenceResponse(
            True, '',
            self.presence_filtered,
            self.presence_raw
        )

    def run(self):
        while not rospy.is_shutdown():
            ret, img = self.cap.read() # capture from camera  
            height, width, _ = img.shape   
            frame_area = width * height

            results = self.detector.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)) # run AprilTag detection on grayscale frame
            timestamp = rospy.Time.now(); time_sec = timestamp.to_sec() # get timestamp of this frame

            def publish_event():
                rospy.loginfo(f'yolo_detector: event: presence changed to {self.presence_filtered}')
                self.pub_event.publish(Event(
                    Header(
                        self.event_seq,
                        timestamp,
                        str(self.frames)
                    ),
                    self.presence_filtered
                ))
                self.event_seq += 1

            if len(results) == 0: # object is in frame
                if not self.presence_raw: # first presence
                    self.presence_raw = True
                    self.presence_timestamp = time_sec
                
                if not self.presence_filtered and time_sec - self.presence_timestamp >= self.enter_min: # object is in frame for long enough
                    self.presence_filtered = True
                    publish_event()
            else: # object is not in frame
                if self.presence_raw: # first non-presence
                    self.presence_raw = False
                    self.presence_timestamp = time_sec
                
                if self.presence_filtered and time_sec - self.presence_timestamp >= self.exit_max: # object is not in frame for long enough
                    self.presence_filtered = False
                    publish_event()

            if self.show_cam: # display detection result
                for r in results:
                    (ptA, ptB, ptC, ptD) = r.corners
                    ptA = (int(ptA[0]), int(ptA[1]))
                    ptB = (int(ptB[0]), int(ptB[1]))
                    ptC = (int(ptC[0]), int(ptC[1]))
                    ptD = (int(ptD[0]), int(ptD[1]))

                    cv2.line(img, ptA, ptB, (0, 255, 0), 2)
                    cv2.line(img, ptB, ptC, (0, 255, 0), 2)
                    cv2.line(img, ptC, ptD, (0, 255, 0), 2)
                    cv2.line(img, ptD, ptA, (0, 255, 0), 2)
                
                cv2.imshow('yolo_detector', img)
                if cv2.waitKey(1) == ord('q'): break
            
            self.frames += 1

                    
if __name__ == '__main__':
    rospy.init_node('apriltag_detector', anonymous=True)

    node_name = rospy.get_name()
    rospy.loginfo(f'apriltag_detector: launching under node namespace {node_name}')

    AprilTagDetector(
        node_name,
        str(rospy.get_param(f'{node_name}/OBJ_NAME')),
        int(rospy.get_param(f'{node_name}/CAM_ID', default=0)),
        str(rospy.get_param(f'{node_name}/CAM_RESOLUTION', default=None)),
        float(rospy.get_param(f'{node_name}/ENTER_MIN', default=1.0)),
        float(rospy.get_param(f'{node_name}/EXIT_MAX', default=0.5)),
        bool(rospy.get_param(f'{node_name}/SHOW_CAM', default=False))
    )