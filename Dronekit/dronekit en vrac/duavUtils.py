import numpy as np
import argparse
import imutils
import time, sys, math
import cv2
import os
from multiprocessing import Process, Pipe

#dronekit imports
from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil

def runDetection(conn):
        
    # initialize a list of colors to represent each possible class label
    COLORS = [(0, 0, 255), (0, 255, 255), (255, 0, 0), (100, 100, 255), (100, 255, 255), (255, 100, 100)]

    conf = 0.5
    thres = 0.2

    # paths to the class labels, model configuration and YOLO weights
    labelsPath = "darknet_cfg/obj.names"
    configPath = "darknet_cfg/yolov3-tiny.cfg"
    weightsPath = "darknet_cfg/yolov3-tiny_10000.weights"

    # load the class labels our YOLO model was trained on
    print("[INFO] loading YOLO from disk...")
    LABELS = open(labelsPath).read().strip().split("\n")
    # load YOLO object detector (cfg and weights)
    net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
    # get all layer names
    allLayersNames = net.getLayerNames()
    # and determine only the *output* layer names that we need from YOLO
    OutLayersNames = [allLayersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    print("[INFO] done")

    rval = False
    # open display window
    # initialize the video stream, and frame dimensions
    cv2.namedWindow("drone's camera")
    videoStream = cv2.VideoCapture(0)
    (W, H) = (None, None)

    if videoStream.isOpened(): # try to get the first frame
        rval, frame = videoStream.read()
        
    while rval:

        # if the frame dimensions are empty, grab them
        if W is None or H is None:
            (H, W) = frame.shape[:2]

        # construct a blob from the input frame and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes
        # and associated probabilities
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (608, 608),
            swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(OutLayersNames)
        end = time.time()

        # initialize our lists of detected bounding boxes, confidences,
        # and class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability)
                # of the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > conf:
                    # scale the bounding box coordinates back relative to
                    # the size of the image, keeping in mind that YOLO
                    # actually returns the center (x, y)-coordinates of
                    # the bounding box followed by the boxes' width and
                    # height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # use the center (x, y)-coordinates to derive the top
                    # and and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # update our list of bounding box coordinates,
                    # confidences, and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping
        # bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf, thres)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping

            for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                # draw a bounding box rectangle and label on the frame
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]],
                    confidences[i])
                cv2.putText(frame, text, (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        #display frame
        cv2.imshow("drone's camera", frame)
        conn.send("detection")

        #read next frame
        rval, frame = videoStream.read()

def readmission(missionFile):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s"%(missionFile))
    missionlist=[]
    with open(missionFile) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
        
def upload_mission(vehicle, missionFile):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(missionFile)
    
    print("\nUpload mission from a file: %s" % missionFile)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print('Upload mission')
    vehicle.commands.upload()

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        
        time.sleep(1)

def send_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def connect_to_drone():

    # Connect to the drone
    print("[INFO] Connecting to drone")
    connection_string = '127.0.0.1:14551'
    vehicle = connect(connection_string, wait_ready=True)

    # Display basic drone state
    print(" Type: %s" %(vehicle._vehicle_type))
    print(" Armed: %s" %(vehicle.armed))
    print(" System status: %s" %(vehicle.system_status.state))
    print(" GPS: %s" %(vehicle.gps_0))
    print(" Alt: %s" %(vehicle.location.global_relative_frame.alt))
    
    return vehicle