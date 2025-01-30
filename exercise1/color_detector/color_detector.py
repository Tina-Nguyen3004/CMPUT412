#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep
import os

# Reference: https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv#:~:text=For%20HSV%2C%20the%20hue%20range,Different%20software%20use%20different%20scales.
N_SPLITS = int(os.getenv("N_SPLITS", 3))
color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [129, 50, 70]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}

def gst_pipeline_string():
    res_w, res_h, fps = 640, 480, 30  # Camera settings
    camera_mode = 3  # Selects the best mode
    gst_pipeline = """ \
        nvarguscamerasrc \
        sensor-mode= exposuretimerange="100000 80000000" ! \
        video/x-raw(memory:NVMM), width=, height=, format=NV12, \
            framerate=/1 ! \
        nvjpegenc ! \
        appsink \
    """.format(
        camera_mode,
        res_w,
        res_h,
        fps
    )

    print("Using GST pipeline: ``".format(gst_pipeline))
    return gst_pipeline

def detect_dominant(sector):
    max_pixels = 0
    dominant_color = "unknown"
    red_pixels = 0 

    for color, (lower, upper) in color_dict_HSV.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(sector, lower, upper)
        count = cv2.countNonZero(mask)

        # Merge red1 and red2
        if color == "red1" or color == "red2":
            red_pixels += count
            continue  

        if count > max_pixels:
            max_pixels = count
            dominant_color = color

    # Check if red is the most dominant color
    if red_pixels > max_pixels:
        dominant_color = "red"

    return dominant_color

        
# cap = cv2.VideoCapture()
# cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)

cap = cv2.VideoCapture("bus.mp4")  # Replace with a video file
cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)
while True:
    ret, frame = cap.read()  # Capture frame
    if not ret:
        print("Failed to capture image")
        break
    
    height, width = frame.shape[:2]
    sector_height = height // N_SPLITS
    for i in range(N_SPLITS):
        sector
        sector = frame[(i*sector_height):((i+1)*sector_height), :]
        hsv = cv2.cvtColor(sector, cv2.COLOR_BGR2HSV)
        dominant_color = detect_dominant(hsv)
        print(f"Sector {i+1}: {dominant_color}\n")

    sleep(1)  # Wait 1 second before next capture

cap.release()
cv2.destroyAllWindows()