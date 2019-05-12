from VideoCapture import Device

import cv2
import numpy

cam = Device()
cam.setResolution(640, 480)

cv2.namedWindow("preview")

while True:
    key = cv2.waitKey(20)
    if key == 27:  # exit on ESC
        break

    frame = numpy.asarray(cam.getImage())
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # display frame
    cv2.imshow("preview", frame)