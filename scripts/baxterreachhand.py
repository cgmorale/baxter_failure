#!/usr/bin/env python
# import roslib
# from roslib import message
# roslib.load_manifest('test_cam')
import rospy
import cv2
import math
import copy
import time
import threading
import argparse
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
# Messages
from geometry_msgs.msg import (
    Point,
    PointStamped,
)
from sensor_msgs.msg import (
    Image,
    PointCloud2,
)
from std_msgs.msg import Header

# Rectangle is a class that stores a rectangle at a particular time.  It is
# used to keep track of the rectangles that the cascade classifier detects
# as hands over time
class Rectangle(object):
    def __init__(self, x, y, w, h, now):
        if math.isnan(x) or math.isnan(y) or math.isnan(w) or math.isnan(h):
            raise Exception("Trying to create coord with nans...%f,%f,%f,%f" % (x,y,w,h))
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.now = now

    def __str__(self):
        return "Rect(x="+str(self.x)+", y="+str(self.y)+", w="+str(self.w)+", h="+str(self.h)+", now="+str(self.now)+")"

    def __repr__(self):
        return self. __str__()

    def getCenter(self):
        return (self.x+self.w/2, self.y+self.h/2)

# A Hand is effectively a collection of PointStamped objects, which corresponding
# to the (x,y,z) of the detected hand at a particular timestamp.
# maxDx, maxDy, and maxDz indicate how far the point could have moved in
# timeInterval nanoseconds
class Hand(object):
    def __init__(self, point, maxDx, maxDy, maxDz):#, timeInterval):
        if type(point) != PointStamped:
            raise TypeException("first argument to hand must be of type rectangle")
        self.positions = [point]
        self.maxDx = maxDx
        self.maxDy = maxDy
        self.maxDz = maxDz

    def __str__(self):
        string = "[\n"
        for point in self.positions:
            string += "     "+str(point)+"\n"
        return string+"]"

    def __repr__(self):
        return self. __str__()

    def couldHaveMovedHere(self, point):
        print("couldHaveMovedHere", point, self.positions[-1])
        pointTime = point.header.stamp.secs*10**9+point.header.stamp.nsecs
        lastPositionTime = self.positions[-1].header.stamp.secs*10**9+self.positions[-1].header.stamp.nsecs
        dtime = float(pointTime-lastPositionTime)/10**9 # convert to secs
        print("dtime", dtime)
        if dtime == 0 or dtime == 0.0:
            return True
        dX = abs(point.point.x-self.positions[-1].point.x)*10**2 # convert to cm
        dY = abs(point.point.y-self.positions[-1].point.y)*10**2 # convert to cm
        dZ = abs(point.point.z-self.positions[-1].point.z)*10**2 # convert to cm
        xRate, yRate, zRate = dX/dtime, dY/dtime, dZ/dtime
        print("rates", xRate, yRate, zRate, self.maxDx, self.maxDy, self.maxDz)
        return (xRate < self.maxDx and yRate < self.maxDy and zRate < self.maxDz)

    # returns whether the rect was appended (if the hand could viably move there).
    # if the average rect was previously calculated, it also returns
    def addPosition(self, point):
        if self.couldHaveMovedHere(point):
            self.positions.append(point)
            return True
        return False

    def getLatestPos(self):
        if len(self.positions) == 0:
            return None
        return self.positions[-1]

    # Gets the average of the last positions that the hand has been in between
    # the current time and interval dTime seconds
    def getAveragePosByTime(self, dTime):
        if len(self.positions) == 0:
            return None, 0
        now = time.time()
        avgX, avgY, avgZ, num = 0, 0, 0, 0
        for point in self.positions[-1::-1]: # reverse order
            if now - point.header.stamp.secs <= dTime:
                avgX += point.point.x
                avgY += point.point.y
                avgZ += point.point.z
                num += 1
            else:
                break # Assume times are in non-decreasing order
        if num == 0:
            return None, num
        avgX /= num
        avgY /= num
        avgZ /= num
        pointMsg = PointStamped()
        pointMsg.header = self.positions[-1].header
        pointMsg.point = Point()
        pointMsg.point.x = avgX
        pointMsg.point.y = avgY
        pointMsg.point.z = avgZ
        return pointMsg, num

class HandDetector(object):

    def __init__(self, topic, rate, cameraName, handModelPath, maxDx, maxDy, maxDz, timeToDeleteHand, groundDzThreshold,
    avgPosDtime, avgHandXYZDtime, maxIterationsWithNoDifference, differenceThreshold, differenceFactor,
    cascadeScaleFactor, cascadeMinNeighbors, handHeightIntervalDx, handHeightIntervalDy, getDepthAtMidpointOfHand, getAveragePos):
        self.bridge = CvBridge()
        self.maxDx = maxDx
        self.maxDy = maxDy
        self.maxDz = maxDz
        self.timeToDeleteHand = timeToDeleteHand
        self.hands_cascade = cv2.CascadeClassifier(handModelPath)
        self.handPositionPublisher = rospy.Publisher(topic, PointStamped, queue_size=1)
        self.groundZ = None
        self.dZ = groundDzThreshold # Ignore all points with height +/- dz of groundz
        self.cascadeScaleFactor = cascadeScaleFactor
        self.cascadeMinNeighbors = cascadeMinNeighbors
        self.avgHandXYZDtime = avgHandXYZDtime
        self.maxIterationsWithNoDifference = maxIterationsWithNoDifference
        self.differenceThreshold = differenceThreshold
        self.differenceFactor = differenceFactor
        self.handHeightIntervalDx = handHeightIntervalDx
        self.handHeightIntervalDy = handHeightIntervalDy
        self.getDepthAtMidpointOfHand = getDepthAtMidpointOfHand
        self.getAveragePos = getAveragePos
        self.avgPosDtime=avgPosDtime
        self.rate=rate

    def start(self):
        self.fgbg = cv2.BackgroundSubtractorMOG()
        self.hands = [] # list of hand objects
        self.handsLock = threading.Lock()
        self.killThreads = False
        self.rgbData = None
        self.rgbDataLock = threading.Lock()
        self.rgbThread = threading.Thread(target=self.cascadeClassifier, args=(self.rate,))
        self.rgbThread.daemon = True
        self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.rgbKinectcallback, queue_size=1)
        self.depth_registered_points_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.depthKinectcallback)
        self.rgbThread.start()
        self.detectedHandsRect = []
        self.detectedHandsRectLock = threading.Lock()
        self.detectedHandsXYZ = []
        self.detectedHandsXYZLock = threading.Lock()
        self.depthData = None
        self.depthDataLock = threading.Lock()
        self.iterationsWithNoDifference = 0
        self.depthThread = threading.Thread(target=self.getHandDepth, args=(self.rate,))
        self.depthThread.daemon = True
        self.depthThread.start()
        self.classifyHandsThread = threading.Thread(target=self.classifyHands, args=(self.rate,))
        self.classifyHandsThread.daemon = True
        self.classifyHandsThread.start()
        self.avgPosThread = threading.Thread(target=self.getPosOfMostLikelyHand, args=(self.avgPosDtime,self.rate))
        self.avgPosThread.daemon = True
        self.avgPosThread.start()

    def stop(self):
        self.rgb_image_sub.unregister()
        self.depth_registered_points_sub.unregister()
        self.killThreads = True
        cv2.destroyAllWindows()

    def rgbKinectcallback(self,data):
        if self.rgbDataLock.acquire(False):
            self.rgbData = data
            self.rgbDataLock.release()

    def cascadeClassifier(self, rate):
        rateMonitor = rospy.Rate(rate)
        try:
            img, oldImg = None, None
            while not rospy.is_shutdown() and not self.killThreads:
                rateMonitor.sleep()
                self.rgbDataLock.acquire(True)
                if self.rgbData is None:
                    self.rgbDataLock.release()
                    continue
                try:
                    if img is not None:
                        oldImg = np.copy(img)
                        #print("reset oldImg")
                    img = self.bridge.imgmsg_to_cv2(self.rgbData, "bgr8")
                    self.rgbDataLock.release()
                except CvBridgeError as e:
                    self.rgbDataLock.release()
                    #print(e)
                    continue
                height, width, channels = img.shape
                #print("share", height, width, channels)
                if oldImg is not None and img is not None:
                    diff = np.nonzero(cv2.subtract(img, oldImg) > self.differenceThreshold)[0].shape
                    #print("diff", diff, height*width*channels, height*width*channels*self.differenceFactor)
                    if diff[0] < height*width*channels*self.differenceFactor:
                        self.iterationsWithNoDifference += 1
                        #print("diff < threshold, iterations", self.iterationsWithNoDifference)
                        if self.iterationsWithNoDifference > self.maxIterationsWithNoDifference:
                            #print("remove hands")
                            self.handsLock.acquire()
                            self.hands = [] # list of hand objects
                            self.handsLock.release()
                            # reset background
                            self.fgbg = cv2.BackgroundSubtractorMOG()
                            self.iterationsWithNoDifference = 0
                    else:
                        self.iterationsWithNoDifference = 0


                fgmask = self.fgbg.apply(img)
                hands = self.hands_cascade.detectMultiScale(fgmask, self.cascadeScaleFactor, self.cascadeMinNeighbors)
                self.detectedHandsRectLock.acquire()
                self.detectedHandsRect = []
                #print("Reset detectedHandsRect")
                for (x,y,w,h) in hands:
                    if not (math.isnan(x) or math.isnan(y) or math.isnan(w) or math.isnan(h)):
                        # self.detectedHandsRectLock.acquire()
                        self.detectedHandsRect.append(Rectangle(x, y, w, h, time.time()))
                        #print("Added rect to detectedHandsRect")
                        rectColor = (255,0,0)
                        rectThickness = 2
                        cv2.rectangle(fgmask,(x,y),(x+w,y+h), rectColor, rectThickness)
                print("detectedHandsRect", self.detectedHandsRect)
                self.detectedHandsRectLock.release()

                cv2.imshow("Image window", fgmask)
                # Number of ms to show the image for
                cv2.waitKey(1)
        except KeyboardInterrupt, rospy.ROSInterruptException:
            cv2.destroyAllWindows()
            return

    def classifyHands(self, rate):
        rateMonitor = rospy.Rate(rate)
        try:
            while not rospy.is_shutdown() and not self.killThreads:
                rateMonitor.sleep()
                self.detectedHandsXYZLock.acquire()
                if len(self.detectedHandsXYZ) == 0:
                    self.detectedHandsXYZLock.release()
                    #print("hand is none")
                    continue
                points = copy.copy(self.detectedHandsXYZ)
                self.detectedHandsXYZ = []
                print("Copied detectedHandsXYZ", points)
                self.detectedHandsXYZLock.release()
                for point in points:
                    self.addToHands(point)
        except KeyboardInterrupt, rospy.ROSInterruptException:
            return

    def addToHands(self, point):
        self.handsLock.acquire()
        i = 0
        print("addToHands", len(self.hands), self.hands)
        while True: # I do this to avoid python calculating the len at the beginning and not accounting for changes to the length through this loop
            if i >= len(self.hands):
                break
            hand = self.hands[i]
            recentPoint = hand.getLatestPos()
            # Lazily remove detected hands if they are too old
            if recentPoint is None or recentPoint.header.stamp.secs <= time.time()-self.timeToDeleteHand:
                print("pop hand", recentPoint, time.time(), time.time()-recentPoint.header.stamp.secs)
                self.hands.pop(i)
                continue
            print("hands", len(self.hands))
            if hand.addPosition(point):
                #print("added to hands 1")
                self.handsLock.release()
                return
            i += 1
        self.hands.append(Hand(point, self.maxDx, self.maxDy, self.maxDz))
        #print("added to hands 2")
        self.handsLock.release()
        return

    def depthKinectcallback(self, data):
        if self.depthDataLock.acquire(False):
            #print("release acqurie 1")
            if self.depthData is None: # First timeToSleep
                # Get ground z
                maxZ = None
                for handCoord in pc2.read_points(data, field_names=None, skip_nans=True):
                    if not (math.isnan(handCoord[2]) or handCoord[2]==0.0):
                        if maxZ is None or handCoord[2] > maxZ:
                            maxZ = handCoord[2]
                self.groundZ = maxZ
                print("groundZ", maxZ)
            self.depthData = data
            #print("release 4")
            self.depthDataLock.release()

    def getHandDepth(self, rate):
        rateMonitor = rospy.Rate(rate)
        try:
            while not rospy.is_shutdown() and not self.killThreads:
                rateMonitor.sleep()
                self.detectedHandsRectLock.acquire()
                if len(self.detectedHandsRect) == 0:
                    self.detectedHandsRectLock.release()
                    #print("no detected hands rects...")
                    continue
                rects = copy.copy(self.detectedHandsRect)
                self.detectedHandsRect = []
                print("Copied detectedHandsRect", rects)
                self.detectedHandsRectLock.release()

                self.detectedHandsXYZLock.acquire()
                self.detectedHandsXYZ = []
                #print("Reset detectedHandsXYZ")

                for rect in rects:
                    self.depthDataLock.acquire(True)
                    #print("release acqurie 2")
                    if self.depthData is None:
                        #print("release 1")
                        self.depthDataLock.release()
                        #print("depth data is None")
                        continue
                    uvs = []
                    avgX, avgY, avgZ, num = float(0),float(0),None,0
                    if self.getDepthAtMidpointOfHand:
                        uvs = [rect.getCenter()]
                    else:
                        dx = self.handHeightIntervalDx
                        dy = self.handHeightIntervalDy
                        midX, midY = rect.getCenter()
                        numXIntervals = rect.w/2/dx
                        numYIntervals = rect.h/2/dy
                        for i in xrange(-1*numXIntervals, numXIntervals+1):
                            for j in xrange(-1*numYIntervals, numYIntervals+1):
                                x = midX + i*dx
                                y = midY + j*dy
                                if (x >= self.depthData.width) or (y >= self.depthData.height):
                                    # #print("hand coord is out of pix")
                                    continue
                                uvs.append((x,y))
                        #print("uvs", uvs)
                    try:
                        data_out = pc2.read_points(self.depthData, field_names=None, skip_nans=False, uvs=uvs)
                    except e:
                        #print(e)
                        self.depthDataLock.release()
                        continue
                    #print("release 3")
                    self.depthDataLock.release()
                    print("uvs", len(uvs))
                    for i in xrange(len(uvs)):
                        try:
                            handCoord = next(data_out)
                            print("handCoord", handCoord)
                        except StopIteration:
                            print("got StopIteration")
                            break
                        if not (math.isnan(handCoord[0]) or handCoord[0]==0.0 or math.isnan(handCoord[1]) or handCoord[1]==0.0 or math.isnan(handCoord[2]) or handCoord[2]==0.0 or handCoord[2] > self.groundZ - self.dZ):
                            avgX += handCoord[0]
                            avgY += handCoord[1]
                            if avgZ is None or handCoord[2] < avgZ:
                                avgZ = handCoord[2] # min not avg
                            num += 1
                        else:
                            print("got handcoord, but not valid (why?)", handCoord, self.groundZ)
                            pass
                    if num == 0:
                        print("got no points with intensity")
                        continue
                    #print("avg in depth", avgX, avgY, avgZ, num)
                    avgX /= float(num)
                    avgY /= float(num)
                    #print("depth", avgX, avgY, avgZ)
                    pointMsg = PointStamped()
                    pointMsg.header = self.depthData.header
                    pointMsg.point = Point()
                    pointMsg.point.x = avgX
                    pointMsg.point.y = avgY
                    pointMsg.point.z = avgZ
                    self.detectedHandsXYZ.append(pointMsg)
                print("detectedHandsXYZ", self.detectedHandsXYZ)
                self.detectedHandsXYZLock.release()
        except KeyboardInterrupt, rospy.ROSInterruptException:
            return

    # TODO (amal): what if instead of determining most likely hand by a
    # majority vote, I augment that with how close the hand is to the center
    # of the robot (in this case I would have to change the voting part to
    # after we get the depth for all hands)?  Might help prevent the robot
    # from going to either the robot arm as a hand, or tthe ground as a hand
    # Gets the average of the last handCoord that the hand has been in between
    # the current time and interval dTime seconds.
    # The reason I don't do this is that in order to do it, we would need to
    # incorporate the robot in this node, but currently this node only
    # interacts with the camera.
    def getPosOfMostLikelyHand(self, dTime, rate):
        rateMonitor = rospy.Rate(rate)
        try:
            while not rospy.is_shutdown() and not self.killThreads:
                rateMonitor.sleep()
                #print("in getAveragePosByTime")
                self.handsLock.acquire()
                if len(self.hands) == 0:
                    #print("len of self.hands is 0", self.hands)
                    self.handsLock.release()
                    continue
                maxNum, maxAvgPoint, maxI = 0, None, None
                for i in xrange(len(self.hands)):
                    hand = self.hands[i]
                    pos, num = hand.getAveragePosByTime(self.avgHandXYZDtime)
                    #print("got avg pos of hand", pos, num)
                    if num > maxNum:
                        maxNum = num
                        maxAvgPoint = pos
                        maxI = i
                self.handsLock.release()
                if maxNum == 0:
                    continue
                if self.getAveragePos:
                    pointToPublish = maxAvgPoint
                else:
                    pointToPublish = self.hands[maxI].getLatestPos()
                self.handPositionPublisher.publish(pointToPublish)
                #print("published pos", maxAvgPoint)
                continue
        except KeyboardInterrupt, rospy.ROSInterruptException:
          return

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', required=True,
        help="the camera name"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    # We don't want multiple instances of this node running
    rospy.init_node('HandDetector', anonymous=False)
    detector = HandDetector(
        topic=rospy.get_param("reachingHand/topic"),
        rate=rospy.get_param("reachingHand/HandDetector/rate"),
        cameraName=args.camera,
        handModelPath=rospy.get_param("reachingHand/HandDetector/handModelPath"),
        maxDx=rospy.get_param("reachingHand/HandDetector/maxAllowedHandMotion/dx"),
        maxDy=rospy.get_param("reachingHand/HandDetector/maxAllowedHandMotion/dy"),
        maxDz=rospy.get_param("reachingHand/HandDetector/maxAllowedHandMotion/dz"),
        timeToDeleteHand=rospy.get_param("reachingHand/HandDetector/timeToDeleteHand"),
        groundDzThreshold=rospy.get_param("reachingHand/HandDetector/groundDzThreshold"),
        avgPosDtime=rospy.get_param("reachingHand/HandDetector/avgPosDtime"),
        avgHandXYZDtime=rospy.get_param("reachingHand/HandDetector/avgHandXYZDtime"),
        maxIterationsWithNoDifference=rospy.get_param("reachingHand/HandDetector/imageDifferenceParams/maxIterations"),
        differenceThreshold=rospy.get_param("reachingHand/HandDetector/imageDifferenceParams/differenceThreshold"),
        differenceFactor=rospy.get_param("reachingHand/HandDetector/imageDifferenceParams/differenceFactor"),
        cascadeScaleFactor=rospy.get_param("reachingHand/HandDetector/cascadeClassifierParams/scale"),
        cascadeMinNeighbors=rospy.get_param("reachingHand/HandDetector/cascadeClassifierParams/minNeighbors"),
        handHeightIntervalDx=rospy.get_param("reachingHand/HandDetector/handHeightInterval/dx"),
        handHeightIntervalDy=rospy.get_param("reachingHand/HandDetector/handHeightInterval/dy"),
        getDepthAtMidpointOfHand=rospy.get_param("reachingHand/HandDetector/getDepthAtMidpointOfHand"),
        getAveragePos=rospy.get_param("reachingHand/HandDetector/getAveragePos"),
    )
    detector.start()
    try:
        rospy.spin()
    except KeyboardInterrupt, rospy.ROSInterruptException:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()