import numpy as np
import argparse
import imutils
import time
import cv2
import os

from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil

def RunDetection(conn):

    COLORS = [(0, 0, 255), (0, 255, 255), (255, 0, 0), (100, 100, 255), (100, 255, 255), (255, 100, 100)]

    conf = 0.5
    thres = 0.2

    labelsPath = "darknet_cfg/obj.names"
    configPath = "darknet_cfg/yolov3-tiny.cfg"
    weightsPath = "darknet_cfg/yolov3-tiny_10000.weights"

    print("[INFO] loading YOLO from disk...")
    LABELS = open(labelsPath).read().strip().split("\n")
    net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
    allLayersNames = net.getLayerNames()
    OutLayersNames = [allLayersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    print("[INFO] done")

    cv2.namedWindow("drone's camera")
    videoStream = cv2.VideoCapture(0)
    writer = None
    (W, H) = (None, None)

    if videoStream.isOpened(): 
        rval, frame = videoStream.read()
    else:
        rval = False


    while rval:

        key = cv2.waitKey(20)
        if key == 27:
            break
        if W is None or H is None:
            (H, W) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (608, 608),
            swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(OutLayersNames)
        end = time.time()

        boxes = []
        confidences = []
        classIDs = []

        for output in layerOutputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if confidence > conf:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf,
            thres)

        if len(idxs) > 0:

            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]],
                    confidences[i])
                cv2.putText(frame, text, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                conn.send(LABELS[classIDs[i]])
                

        cv2.imshow("drone's camera", frame)
        rval, frame = videoStream.read()

    conn.send(None)
    print("[INFO] cleaning up...")
    cv2.namedWindow("preview")
    videoStream.release()

def connect_to_drone():

    print("[INFO] Connecting to drone")
    connection_string = '127.0.0.1:14551'
    vehicle = connect(connection_string, wait_ready=True)

    print(" Type: %s" %(vehicle._vehicle_type))
    print(" Armed: %s" %(vehicle.armed))
    print(" System status: %s" %(vehicle.system_status.state))
    print(" GPS: %s" %(vehicle.gps_0))
    print(" Alt: %s" %(vehicle.location.global_relative_frame.alt))
    
    return vehicle

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)