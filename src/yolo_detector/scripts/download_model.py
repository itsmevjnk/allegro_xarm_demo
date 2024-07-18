#!/usr/bin/env python3

import rospy
import rospkg
import requests
from tqdm import tqdm

URL='https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt'
MODEL=rospkg.RosPack().get_path('yolo_detector') + '/yolov8n.pt'

if __name__ == '__main__':
    print(f'Downloading pretrained model to {MODEL}...')
    resp = requests.get(URL, stream=True)
    with open(MODEL, 'wb') as f:
        for data in tqdm(resp.iter_content()):
            f.write(data)