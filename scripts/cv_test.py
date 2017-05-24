#!/usr/bin/env python
import cv2
import cv2 as cv

capture = cv2.VideoCapture(cv2.CAP_OPENNI)
capture.set(cv2.CAP_OPENNI_DEPTH_GENERATOR, cv2.CAP_OPENNI_VGA_30HZ)
print capture.get(cv2.CAP_PROP_OPENNI_REGISTRATION)

while True:
    if not capture.grab():
        print "Unable to Grab Frames from camera"
        break
    okay1, depth_map = capture.retrieve(cv2.CAP_OPENNI_DEPTH_MAP)
    if not okay1:
        print "Unable to Retrieve Disparity Map from camera"
        break
    okay2, gray_image = capture.retrieve(cv2.CAP_OPENNI_GRAY_IMAGE)
    if not okay2:
        print "Unable to retrieve Gray Image from device"
        break
    cv2.imshow("depth camera", depth_map)
    cv2.imshow("rgb camera", gray_image)
    if cv2.waitKey(10) == 27:
        break
cv2.destroyAllWindows()
capture.release()
