#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO

from std_msgs.msg import Header
from yolo_detector.msg import Event
from yolo_detector.srv import GetPresence, GetPresenceResponse

YOLO_MODEL = '/home/itsmevjnk/yolov8n.pt'

class YOLODetector:
    def __init__(self, namespace: str, cam_id: int, classifiers: 'list[str]', cam_resolution: 'str | None' = None, detect_bounds: 'list[float]' = [0,0,1,1], conf_thres: float = 0.6, area_thres: float = 0.0, enter_min: float = 1.0, exit_max: float = 0.5, show_cam: bool = False):
        self.cap = cv2.VideoCapture(cam_id) # start camera
        if cam_resolution is not None and len(cam_resolution) > 0: # set custom resolution
            cam_resolution = cam_resolution.split('x')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam_resolution[0]))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam_resolution[1]))

        self.model = YOLO(YOLO_MODEL, verbose=False) # load model

        # get list of classifiers to look for
        self.classifier_names = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
        self.classifiers = {self.classifier_names.index(cls): [False, False, 0.0] for cls in classifiers} # raw presence, filtered presence, timestamp since first presence (or first non-presence)
        rospy.loginfo(f'yolo_detector: classifiers to look for: {self.classifiers}')

        # save other args
        self.conf_thres = conf_thres
        self.area_thres = area_thres
        self.enter_min = enter_min
        self.exit_max = exit_max
        self.show_cam = show_cam
        self.boundary = detect_bounds

        # start interface
        rospy.loginfo('yolo_detector: starting ROS interface')
        self.pub_event = {cls: [rospy.Publisher(f'{namespace}/events/{self.classifier_names[cls]}', Event, queue_size=10), 0] for cls in self.classifiers} # publisher, seq
        self.srv_presence = {cls: rospy.Service(f'{namespace}/pres/{self.classifier_names[cls]}', GetPresence, lambda msg, cls_=cls: self.srv_presence_cb(cls_, msg)) for cls in self.classifiers}

        self.frames = 0 # frame counter
        self.run() # start running

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def srv_presence_cb(self, cls, msg):
        return GetPresenceResponse(
            True, '',
            self.classifiers[cls][1],
            self.classifiers[cls][0]
        )

    def run(self):
        xmin, ymin, xmax, ymax = self.boundary
        while not rospy.is_shutdown():
            ret, img = self.cap.read() # capture from camera  
            height, width, _ = img.shape   
            frame_area = width * height

            results = self.model(img, stream=True, verbose=False) # run model on this frame
            timestamp = rospy.Time.now(); time_sec = timestamp.to_sec() # get timestamp of this frame

            in_frame = [] # list of (valid) classifiers in our frame

            for r in results:
                for box in r.boxes: # iterate through each object
                    cls = int(box.cls[0]) # get classifier ID
                    if not cls in self.classifiers: continue # not an object of interest

                    if box.conf[0] < self.conf_thres: continue # ignore junk

                    # get bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    xc = (x1 + x2) / 2 / width; yc = (y1 + y2) / 2 / height # calculate centre coordinates
                    if xc < xmin or xc > xmax or yc < ymin or yc > ymax: continue # center is out of bounds

                    # check if bounding box area is large enough
                    if abs(x1 - x2) * abs(y1 - y2) / frame_area < self.area_thres: continue

                    in_frame.append(cls) # add classifier to our list

                    if self.show_cam: # draw bounding box on frame
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(img, f'{self.classifier_names[cls]},{box.conf[0]}', [x1, y1], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            def publish_event(cls):
                rospy.loginfo(f'yolo_detector: event: {self.classifier_names[cls]} presence changed to {self.classifiers[cls][1]}')
                self.pub_event[cls][0].publish(Event(
                    Header(
                        self.pub_event[cls][1],
                        timestamp,
                        str(self.frames)
                    ),
                    self.classifiers[cls][1]
                ))
                self.pub_event[cls][1] += 1

            # iterate through objects of interest to update their state
            for cls in self.classifiers:
                if cls in in_frame: # object is in frame
                    if not self.classifiers[cls][0]: # first presence
                        self.classifiers[cls][0] = True
                        self.classifiers[cls][2] = time_sec
                    
                    if not self.classifiers[cls][1] and time_sec - self.classifiers[cls][2] >= self.enter_min: # object is in frame for long enough
                        self.classifiers[cls][1] = True
                        publish_event(cls)
                else: # object is not in frame
                    if self.classifiers[cls][0]: # first non-presence
                        self.classifiers[cls][0] = False
                        self.classifiers[cls][2] = time_sec
                    
                    if self.classifiers[cls][1] and time_sec - self.classifiers[cls][2] >= self.exit_max: # object is not in frame for long enough
                        self.classifiers[cls][1] = False
                        publish_event(cls)

            if self.show_cam:
                cv2.rectangle(img, (int(self.boundary[0] * width), int(self.boundary[1] * height)), (int(self.boundary[2] * width), int(self.boundary[3] * height)), (255, 0, 0), 2) # draw boundary for debugging
                cv2.imshow('yolo_detector', img)
                if cv2.waitKey(1) == ord('q'): break
            
            self.frames += 1

                    
if __name__ == '__main__':
    rospy.init_node('yolo_detector', anonymous=True)

    node_name = rospy.get_name()
    rospy.loginfo(f'yolo_detector: launching under node namespace {node_name}')

    YOLODetector(
        node_name,
        int(rospy.get_param(f'{node_name}/CAM_ID', default=0)),
        str(rospy.get_param(f'{node_name}/CLASSIFIERS')).split(','),
        str(rospy.get_param(f'{node_name}/CAM_RESOLUTION', default=None)),
        [float(x) for x in str(rospy.get_param(f'{node_name}/BOUNDARY')).split(',')],
        float(rospy.get_param(f'{node_name}/CONF_THRESHOLD', default=0.5)),
        float(rospy.get_param(f'{node_name}/AREA_THRESHOLD', default=0.0)),
        float(rospy.get_param(f'{node_name}/ENTER_MIN', default=1.0)),
        float(rospy.get_param(f'{node_name}/EXIT_MAX', default=0.5)),
        bool(rospy.get_param(f'{node_name}/SHOW_CAM', default=False))
    )