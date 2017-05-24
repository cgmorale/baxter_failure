#!/usr/bin/env python
from __future__ import print_function
from Tkinter import * 
import numpy as np
import roslib
roslib.load_manifest('baxter_failure')
import sys
import rospy
import cv2
import time 
import glob 
from baxter_core_msgs.msg import DigitalIOState, EndpointState
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, Range 
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
import os.path
import baxter_interface
import errno
import argparse
import struct
import math
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class gripperCameraFeed(object):

    def __init__(self, limb='left'):

        self.pub=rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

        self.limb = limb
        self.cameraPath = "/cameras/%s_hand_camera/image" % (self.limb) 
        #which gripper camera is being viewed
        self.imageSub = rospy.Subscriber(self.cameraPath, Image,self.colorFilter)
        #subscribe to receive data from camera
        self.bridge = CvBridge()
        self.minBlockArea = 500
        self.cXpixel, self.cYpixel = None, None 
    
    def colorFilter(self, data):
        #boundaries = [([110, 50, 50], [130, 255,255])] #blue filter 
                                      #130
        boundaries = [
                    ([100, 50, 50], [130, 255,255]), #blue filter 
                    # ([20, 50, 70], [100, 255, 255]), #green filter
                    # ([0, 100, 240], [255, 255, 255]), #red filter 
                    # ([0, 180, 200], [230, 255, 255]), #yellow filter NOPE 
                    #  ([130, 130, 130], [255, 255, 255]) #white filter NOPE
                    ] 

        try:
            self.pub.publish(data)
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e: print(e)

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper) #mask for color
        #blur = cv2.GaussianBlur(mask, (3, 3), 0)
        #output = cv2.bitwise_and(hsv, hsv, mask = blur) #apply mask to img
        contours= cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #find the contours
        cnt = contours[0];
        M = cv2.moments(cnt)

        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])

        self.cXpixel, self.cYpixel = cX, cY
        

        """

        for c in contours:
            area = cv2.contourArea(contours[0])
            M = cv2.moments(c)

            if(area > self.minBlockArea):
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                self.cXpixel, self.cYpixel = cX, cY
                # print(cX, cY)

                #cv2.drawContours(img,contours,i,(0,0,255),3)#draw contour and circle on center of image 
                cv2.circle(img, (cX, cY), 20, (0, 0, 0), 2) # draw black circle
                center_circle = cv2.circle(img, (cX, cY), 20, (0, 0, 0), 1) #center circle on image 
                cv2.putText(img, "Center", (cX - 20, cY - 20), #put text on circle 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
                #optimal pixel coordinate: 382, 13(1-4) in 640x400 image  
        #cv2.imshow("Color Detection", np.hstack([img, output]))
        """

        cv2.imshow("Stuff", img)
        cv2.waitKey(1)


class Locomotion(gripperCameraFeed):
    """
    important note: The different towers are placed 28 cm apart, 
    approx one 11 x 8.5 sheet of paper for ease of resetting environment

    """
    
    defaultPosition = None
    yStartingPose = None
    interimPose = None
    interimJointAngles = None 
    #all of the above attributes get set in the self.setArmDefaultAttributes()

    cubeHeight = .04 #meters
    towerSpacing = .28 #meters
    grabHeight = -0.167
    #z required to pick up single block resting on table, from endpoint_pose()


    def __init__(self):
        gripperCameraFeed.__init__(self) #grab init data from parent class
        
        #paths for subscription
        circleButtonPath = "/robot/digital_io/%s_lower_button/state" % self.limb
        ovalButtonPath = "/robot/digital_io/%s_upper_button/state" % self.limb
        irRangePath = "/robot/range/%s_hand_range/state" % self.limb

        #subscription/api module instances
        self.gripper = baxter_interface.Gripper(self.limb)
        self.gripperLimb = baxter_interface.Limb(self.limb)
        self.irSensor = rospy.Subscriber(irRangePath, Range, self.IRrangeSensor,
                                                        callback_args=self.limb)
        self.circleButton = rospy.Subscriber(circleButtonPath, DigitalIOState, 
                            self.openGripper, callback_args=self.limb)
        self.ovalButton = rospy.Subscriber(ovalButtonPath, DigitalIOState, 
                                self.closeGripper, callback_args = self.limb)
 
        #administrative instances
        self.maxBlockCenterCalls = 10 
        self.gripSpace = .05 #m, gives clearance for gripping so only single block moved 
        #if block center not found after n attempts give up 
        self.gripperOpen = True 
        self.IRrange = None #reading from wrist ir sensor 
        self.torque = None
        #set to true to print out new default arm position, save to class var
        self.resetDefaultPositions = False
        self.armReady = False #waiting to be in default position 
        self.centerOffsetTolerance = .10 #percent, max center offset tolerance
        self.placeBlockGap = 0.002 #m



    def getBlockCenter(self, attempts=0):
        #get the center x and y coordinates
        if attempts == self.maxBlockCenterCalls:
            #give up after x attempts to locate center 
            print('Cannot find block center. Giving up')
            return False 
        if self.cXpixel != None and self.cYpixel != None:
            return (self.cXpixel, self.cYpixel)
        else:
            time.sleep(.5) #pause method to avoid stack overflow 
            self.getBlockCenter(attempts+1)

    def getCurrentArmPosition(self):
        #returns dictionary of current joint angles for the selected arm
        #print(self.gripperLimb.endpoint_pose()) 
        jointAngles = self.gripperLimb.joint_angles()
        pose = self.gripperLimb.endpoint_pose()
        return (jointAngles, pose)

    def setArmDefaultPosition(self):
        if self.resetDefaultPositions:
            #reset arm default position
            print(self.getCurrentArmPosition())
        else:
           #set arm to default position 
           self.gripperLimb.move_to_joint_positions(Locomotion.defaultPosition, 
                                        timeout=15.0, threshold=0.003726646)
        return

    def setDefaultAttributes(self):
        Locomotion.defaultPosition, pose = self.getCurrentArmPosition()
        Locomotion.yStartingPose = pose['position'].y
        findBlockCenter = self.getBlockCenter()
        print(findBlockCenter)
        if findBlockCenter != False or None:
            self.idealBlockCx, self.idealBlockCy = findBlockCenter
        else:
            return 3%0;

        # self.idealBlockCx, self.idealBlockCy = None, None #pixel goal for gripper alignment 

        # self.centerOffsetTolerance = .10 #percent, max center offset tolerance
        cX, cY = self.idealBlockCx,self.idealBlockCy
        delta = self.centerOffsetTolerance

        self.blockCxRange = list(range(int(cX - cX*delta), int(cX + cX*delta)))
        self.blockCyRange = list(range(int(cY - cY*delta), int(cY + cY*delta)))

        newPos = [pose['position'].x, pose['position'].y - Locomotion.towerSpacing,
                  pose['position'].z]
        quat =  [pose['orientation'].x,pose['orientation'].y,
                    pose['orientation'].z,pose['orientation'].w]
        Locomotion.interimJointAngles = self.IKsolver(newPos, quat, self.limb)
        #new interim Joint Angles 
        self.moveGripper(Locomotion.interimJointAngles)
        #move to interim position
        Locomotion.interimPose = self.getCurrentArmPosition()[1]

       
        # sets new interim arm Pose 
        #now code may proceed as usual 
        return 
        

    def calibrateGripper(self):
        #calibrate gripper and close it 
        self.gripper.calibrate(block=True, timeout=2.0)
        self.gripper.command_position(position=100.0, block =True, timeout=5.0)
        return

    def openGripper(self, msg, side): 
        #0 = not pressed, 1 = pressed
        side = self.limb
        if(msg.state == 1):
            #opens the gripper
            self.gripper.command_position(position=100.0, block =True,timeout=5.0)
        self.gripperOpen = True 
        return

    def closeGripper(self, msg, side):
        #controlled by oval wrist button
        side = self.limb
        if msg.state == 1:
            #closes the gripper
            self.gripper.command_position(position=0.0, block =True,timeout=5.0)
        self.gripperOpen = False
        return 

    def IRrangeSensor(self, msg, side):
        side = self.limb
        self.IRrange = msg.range
        return 

    def torqueDetection(self):
        self.torque = self.gripperLimb.endpoint_effort()
        return self.torque

    def realignGripper(self, cX, cY):
        #pixelToMeter = 0.0005 
        pixelToMeter = 0.000264583
        xPixelOffSet = cX - self.idealBlockCx
        yPixelOffSet = cY - self.idealBlockCy

        xAxisOffset = pixelToMeter*xPixelOffSet 
        yAxisOffset = pixelToMeter*yPixelOffSet

        pose = self.gripperLimb.endpoint_pose() 
        newPos = [pose['position'].x, pose['position'].y, pose['position'].z]
        # ATTENTION. Is it necessary to declare newPos here^^^^^
        quat =  [pose['orientation'].x,pose['orientation'].y,
                    pose['orientation'].z,pose['orientation'].w]
        #newPos to align gripper with block 
        newPos = [pose['position'].x + xAxisOffset, 
                  pose['position'].y - yAxisOffset, 
                  pose['position'].z
                  ]
        jointSolution = self.IKsolver(newPos, quat, self.limb)
        print('moving for realignment')
        self.moveGripper(jointSolution)
        return 


    def moveGripperYposition(self, grabStack, grabStackHeight):
        print('moving to Y position')
        #moves gripper to Y-axis position to prepare for grab
        pose = self.gripperLimb.endpoint_pose() 
        pos, quat = self.modifyReachPose(grabStack, grabStackHeight, pose, False, 'y')
        jointSolution = self.IKsolver(pos, quat, self.limb)
        self.moveGripper(jointSolution)
        return 

    def moveToGrabBlock(self, grabStack, grabStackHeight):
        pose = self.gripperLimb.endpoint_pose()
        pos, quat = self.modifyReachPose(grabStack, grabStackHeight, pose, True, None)
        jointSolution = self.IKsolver(pos, quat, self.limb)
        self.moveGripper(jointSolution)
        self.handleBlock(True) #grab block
        return 

    def moveToPlaceBlock(self, placeStack, placeStackHeight):
        #pose = self.gripperLimb.endpoint_pose() 
        pose = Locomotion.interimPose
        pos, quat = self.modifyReachPose(placeStack, placeStackHeight, pose)
        jointSolution = self.IKsolver(pos, quat, self.limb)
        self.moveGripper(jointSolution)
        self.handleBlock(False) #drop block 
        return 


    def readInstuctions(self, index=0, bypass=False,depth=0):
        #loops through instructions,             
        if index < len(solveTowers.instructions):
            move = solveTowers.instructions[index]
            grabStack, grabStackHeight, placeStack, placeStackHeight = move

            if not bypass:
            #if gripper realigned don't move to default poisition  
                self.moveGripperYposition(grabStack, grabStackHeight)

            blockCx, blockCy = self.getBlockCenter() #get center of block 
            if depth == 5:
                blockCx, blockCy = self.idealBlockCx, self.idealBlockCy
            print('ideal block coords = ',  (self.idealBlockCx, self.idealBlockCy))
            print('block coords = ', (blockCx, blockCy))

            if blockCx in self.blockCxRange and blockCy in self.blockCyRange:
                print('Acceptable Cx, Cy')
                #move gripper on z-axis to grab block 
                self.moveToGrabBlock(grabStack, grabStackHeight)
                
                self.moveGripper(Locomotion.interimJointAngles) 
                #move gripper to interim position 

                self.moveToPlaceBlock(placeStack, placeStackHeight)
            
                self.moveGripper(Locomotion.interimJointAngles)
                #reset to interim position
                self.readInstuctions(index + 1, depth=0) 
                #perform next move recursively 
            else:
                #use IK solver for adjustment as opposed to just going to next stack
                print('Realigning')
                self.realignGripper(blockCx, blockCy)
                self.readInstuctions(index, bypass=True, depth=depth+1) 
                #gripper realigned, redo move recursively 
                

    def modifyReachPose(self, stack, stackHeight, pose, grab=False, axis=None):
        #1 pixel = 0.000264583
        #modifies pose for IK
        if grab: 
            zAxisReach = Locomotion.grabHeight + (stackHeight-1)*Locomotion.cubeHeight 
        else: 
            #placing block
            zAxisReach = Locomotion.grabHeight + stackHeight*Locomotion.cubeHeight 
            zAxisReach += self.placeBlockGap 
            #gripper pushing on  stack = :(, so give baxter space to place block

            #corrects for varying stackHeights

        yAxisReach = Locomotion.yStartingPose - stack*Locomotion.towerSpacing

        #get desired position for y axis   
        newPos = [pose['position'].x, pose['position'].y, pose['position'].z]
        quat =  [pose['orientation'].x,pose['orientation'].y,
                    pose['orientation'].z,pose['orientation'].w]

        if axis == 'y':
            print('y selected')
            newPos = [pose['position'].x, yAxisReach, pose['position'].z]
        elif axis == 'z':
            newPos = [pose['position'].x, pose['position'].y, 
                                        pose['position'].z + self.gripSpace]
        else: 
            newPos = [pose['position'].x, yAxisReach, zAxisReach]
            # ATTENTION. WHY IS IT NECESSARY TO MOVE Y-AXIS ASSUMING BLOCKCX + CY ALIGNED???????????

        return newPos, quat 
    
        
    def IKsolver(self, pos, quat, limb): 
        """
        #CORE OF FUNCTION IS FROM /BAXTER_EXAMPLES/IK_SERVICE_CLIENT.PY
        returns jointSolution for desired pose 
        """
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x = pos[0],
                        y = pos[1],
                        z = pos[2],
                    ),
                    orientation=Quaternion(
                        x = quat[0],
                        y = quat[1],
                        z = quat[2],
                        w = quat[3],
                    ),
                ),
            ),
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=pos[0],
                        y=pos[1],
                        z=pos[2],
                    ),
                    orientation=Quaternion(
                        x=quat[0],
                        y=quat[1],
                        z=quat[2],
                        w=quat[3],
                    ),
                ),
            ),
        }

        ikreq.pose_stamp.append(poses[limb])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # reformat the solution arrays into a dictionary
            joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))

            print("IK_Solver Done")
            return joint_solution

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        return False 

    def handleBlock(self, grab):
        if grab: 
            #closes gripper
            self.gripper.command_position(position=0.0, block =True,timeout=5.0)
        else: 
            #opens gripper
            self.gripper.command_position(position=100.0, block =True,timeout=5.0)
          #  time.sleep(.2)
        return 
    
    def moveGripper(self, jointSolution):
        #takes in instructions from IK solver and moves arm 
        #blocking, use set_joint_position to interrupt (unreliable)
        self.gripperLimb.move_to_joint_positions(jointSolution, threshold=0.005726646)
        print('Torque = ', self.torqueDetection())
        return 

class solveTowers(object):

    instructions = []
    #stores instructions for solving tower 
    #format = [(grabFrom, grabStackHeight, placeHere, placeStackHeight)]

    def __init__(self, stack):
       self.stack = stack 
       self.source = 0
       self.target = 1
       self.temp = 2 

       #changeStackHeightsMethod will use vals to calc. stack heights 
       self.sourceStack = stack
       self.targetStack = 0 
       self.tempStack = 0 

    def changeStackHeights(self, grabFrom, placeHere):
        #manipulate stack heights for 'instructions' class attribute
        if grabFrom == self.source:
            self.sourceStack -= 1  
            if placeHere == self.target: 
                self.targetStack += 1
                return (self.sourceStack + 1, self.targetStack - 1)
            self.tempStack += 1 
            return (self.sourceStack + 1, self.tempStack - 1)
        elif grabFrom == self.temp:
            self.tempStack -= 1
            if placeHere == self.target: 
                self.targetStack += 1
                return (self.tempStack + 1, self.targetStack -1) 
            self.sourceStack += 1 
            return (self.tempStack + 1, self.sourceStack -1)
        else:
            self.targetStack -= 1
            if placeHere == self.source: 
                self.sourceStack += 1
                return (self.targetStack + 1, self.sourceStack -1)
            self.tempStack += 1 
            return (self.targetStack + 1, self.tempStack - 1)

    def hanoi(self):
        #wrapper for solver 
        self.move(self.stack, self.source, self.target, self.temp)

    def move(self, stack, source, target, temp):
        
        if stack == 1:
            sourceHeight, targetHeight = self.changeStackHeights(source, target)
            #get stackHeights
            solveTowers.instructions += [(source, sourceHeight,
                                             target, targetHeight)]
            #add to growing instructions list 
        else:
            self.move(stack-1, source, temp, target)
            self.move(1, source, target, temp)
            self.move(stack-1, temp, target, source)




def main(args):
    rospy.init_node("gripperCameraFeed", anonymous=True)

    cam = gripperCameraFeed() 
    time.sleep(.5)
    solve = solveTowers(1)
    solve.hanoi()
    move = Locomotion()
    move.setDefaultAttributes()
    move.setArmDefaultPosition()
    move.calibrateGripper()
    move.readInstuctions()

    
    


    try:    
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



