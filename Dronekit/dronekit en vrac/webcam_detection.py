import numpy as np
import argparse
import imutils
import time
import cv2
import os

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-y", "--yolo", required=True,
    help="base path to YOLO directory")
ap.add_argument("-c", "--confidence", type=float, default=0.5,
    help="minimum probability to filter weak detections")
ap.add_argument("-t", "--threshold", type=float, default=0.3,
    help="threshold when applying non-maxima suppression")
ap.add_argument("-w", "--weight", required=True,
    help="weight file")
args = vars(ap.parse_args())

# load the class labels our YOLO model was trained on
labelsPath = os.path.sep.join([args["yolo"], "obj.names"])
LABELS = open(labelsPath).read().strip().split("\n")

# initialize a list of colors to represent each possible class label
COLORS = [(0, 0, 255), (0, 255, 255), (255, 0, 0), (50, 50, 255), (50, 255, 255), (255, 50, 50)]

# derive the paths to the YOLO weights and model configuration
configPath = os.path.sep.join([args["yolo"], "yolov3-tiny.cfg"])
weightsPath = os.path.sep.join([args["yolo"], args["weight"]])

# load our YOLO object detector (cfg and weights)
# and determine only the *output* layer names that we need from YOLO
print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)
ln = net.getLayerNames()
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# open display window
# initialize the video stream, pointer to output video file, and
# frame dimensions
cv2.namedWindow("preview")
vs = cv2.VideoCapture(1)
vs.read()

writer = None
(W, H) = (None, None)

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
    blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (832, 832),
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
    
    # apply non-maxima suppression to suppress weak, overlapping
    # bounding boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, args["confidence"],
        args["threshold"])
 
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

#   # check if the video writer is None
#   if writer is None:
#       # initialize our video writer
#       fourcc = cv2.VideoWriter_fourcc(*"MJPG")
#       writer = cv2.VideoWriter(args["output"], fourcc, 30,
#           (frame.shape[1], frame.shape[0]), True)
# 
#       # some information on processing single frame
#       if total > 0:
#           elap = (end - start)
#           print("[INFO] single frame took {:.4f} seconds".format(elap))
#           print("[INFO] estimated total time to finish: {:.4f}".format(
#               elap * total))
# 
#   # write the output frame to disk
#   writer.write(frame)
 
# release the file pointers
print("[INFO] cleaning up...")
cv2.namedWindow("preview")
#writer.release()
vs.release()