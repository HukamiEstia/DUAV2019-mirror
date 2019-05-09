import numpy as np
import argparse
import imutils
import time, sys, math
import cv2
import os

#dronekit imports
from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil

def arm_and_takeoff(aTargetAltitude):
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

def send_local_ned_velocity(vx, vy, vz):
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

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-y", "--yolo", required=True,
    help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.1,
    help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0,
    help="threshold when applying non-maxima suppression")
args = vars(ap.parse_args())

# load the class labels our YOLO model was trained on
labelsPath = os.path.sep.join([args["yolo"], "obj.names"])
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
COLORS = [(0, 0, 255), (0, 255, 255), (255, 0, 0), (100, 100, 255), (100, 255, 255), (255, 100, 100)]

# derive the paths to the YOLO weights and model configuration
configPath = os.path.sep.join([args["yolo"], "yolov3-tiny.cfg"])
weightsPath = os.path.sep.join([args["yolo"], "yolov3-tiny_10000.weights"])

# load our YOLO object detector (cfg and weights)
# and determine only the *output* layer names that we need from YOLO
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
ln = net.getLayerNames()
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Connect to the Vehicle
print "[INFO] Connecting to vehicle"
connection_string = '127.0.0.1:14551'
vehicle = connect(connection_string, wait_ready=True)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

#arm and take off drone
arm_and_takeoff(10)

# open display window
# initialize the video stream, pointer to output video file, and
# frame dimensions
cv2.namedWindow("preview")
vs = cv2.VideoCapture(0)
writer = None
(W, H) = (None, None)

# try to determine the total number of frames in the video file
try:
    prop = cv2.cv.CV_CAP_PROP_FRAME_COUNT if imutils.is_cv2() \
        else cv2.CAP_PROP_FRAME_COUNT
    total = int(vs.get(prop))
    print("[INFO] {} total frames in video".format(total))
 
# an error occurred while trying to determine the total
# number of frames in the video file
except:
    print("[INFO] could not determine # of frames in video")
    print("[INFO] no approx. completion time can be provided")
    total = -1

if vs.isOpened(): # try to get the first frame
    rval, frame = vs.read()
else:
    rval = False

# loop over frames from the video file stream
while rval:
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

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
    layerOutputs = net.forward(ln)
    end = time.time()
 
    # initialize our lists of detected bounding boxes, confidences,
    # and class IDs, respectively
    boxes = []
    confidences = []
    classIDs = []
    red_cross = False


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
            if confidence > args["confidence"]:
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

                # store position of red cross
                if classID == 0:
                    red_cross = True
                    X, Y = centerX, centerY
    
    # apply non-maxima suppression to suppress weak, overlapping
    # bounding boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
        args["threshold"])
    
    if red_cross:
        print(X, Y)
        if (252 <= Y <= 352) and (252 <= X <= 352):
            send_local_ned_velocity(0,0,0.5)
            time.sleep(1)
            print("LANDING")
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        elif Y <= 608 - X and Y < X:
            send_local_ned_velocity(10,0,0)
            time.sleep(1)
            print("moving NORTH")
        elif Y <= 608 - X and Y > X:
            send_local_ned_velocity(0,-10,0)
            time.sleep(1)
            print("moving WEST")
        elif Y >= 608 - X and Y > X:
            send_local_ned_velocity(-10,0,0)
            time.sleep(1)
            print("moving SOUTH")
        elif Y >= 608 - X and Y < X:
            send_local_ned_velocity(0,10,0)
            time.sleep(1)
            print("moving EAST")

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
    cv2.imshow("preview", frame)

    #read next frame
    rval, frame = vs.read()

print("[INFO] cleaning up...")
cv2.namedWindow("preview")
#writer.release()
vs.release()