#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep

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

cap = cv2.VideoCapture()
cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()  # Capture frame
    if not ret:
        print("Failed to capture image")
        break

    # Process the image using NumPy (color detection, etc.)
    # Your magic here

    sleep(1)  # Wait 1 second before next capture
