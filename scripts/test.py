#!/usr/bin/env python
#Obi Adubor
"""
Traditional Towers of Hanoi only has 3 towers, but in my ToH you can 
theoretically have n numbers of temporary/transfer towers. However, since my
robot's arms are not infinitely long I will cap the extra # of temporary 
towers at 5 for the robot's solo solve mode
For the vs. mode (under development) the towers will be arranged normally 
that way the robot can focus on messing with the user as much as possible
"""

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
import time
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

##############################
#Robot Vision Code 
#############################

class gripperCameraFeed(object):

    def __init__(self, limb='left'):

        self.limb = limb
        self.cameraPath = "/cameras/%s_hand_camera/image" % (self.limb) 
        #which gripper camera is being viewed
        self.imageSub = rospy.Subscriber(self.cameraPath, Image,self.colorFilter)
        #subscribe to receive data from camera
        self.bridge = CvBridge()
        self.minBlockArea = 500
        self.cXpixel, self.cYpixel = None, None 
    
    def colorFilter(self, data):
        boundaries = [
                    ([100, 50, 50], [130, 255,255]), #blue filter 
                    ] 
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e: print(e)

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper) #mask for color
        blur = cv2.GaussianBlur(mask, (3, 3), 0)
        output = cv2.bitwise_and(hsv, hsv, mask = blur) #apply mask to img
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) 
        #find the contours

        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            M = cv2.moments(c)

            if(area > self.minBlockArea):
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                self.cXpixel, self.cYpixel = cX, cY
                cv2.drawContours(img,contours,i,(0,0,255),3)
                #draw contour and circle on center of image 
                cv2.circle(img, (cX, cY), 20, (0, 0, 0), 2) 
                #draw black circle
                center_circle = cv2.circle(img, (cX, cY), 20, (0, 0, 0), 1) 
                #center circle on image 
                cv2.putText(img, "Center", (cX - 20, cY - 20), 
                #put text on circle 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
        cv2.imshow("Color Detection", np.hstack([img, output]))
        cv2.waitKey(1) 

###################
#Robot Kinematics
##################

class Locomotion(gripperCameraFeed):
    #Robot Locomotion
    defaultPosition = None
    yStartingPose = None
    interimPose = None
    interimJointAngles = None 
    #all of the above attributes get set in the self.setArmDefaultAttributes()

    cubeHeight = .04 #meters
    towerSpacing = .10 #meters
    grabHeight = -0.17
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
        self.maxBlockCenterCalls = 5 
        self.gripSpace = .05 
        #m, gives clearance for gripping so only single block moved 
        self.gripperOpen = True 
        self.IRrange = None #reading from wrist ir sensor 
        self.torque = None
        #set to true to print out new default arm position, save to class var
        self.resetDefaultPositions = False
        self.armReady = False #waiting to be in default position 
        self.centerOffsetTolerance = .10 #percent, max center offset tolerance
        self.placeBlockGap = 0.003 #2, min. robot's tendency to push on stack

    def getBlockCenter(self, attempts=0):
        #get the center x and y coordinates
        if attempts == self.maxBlockCenterCalls:
            #give up after 5 attempts to locate center 
            return False 
        if self.cXpixel != None or self.cYpixel != None:
            return (self.cXpixel, self.cYpixel)
        else:
            time.sleep(.5) #pause method to avoid stack overflow 
            self.getBlockCenter(attempts+1)

    def getCurrentArmPosition(self):
        #returns dictionary of current joint angles for the selected arm
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
        self.idealBlockCx, self.idealBlockCy = self.getBlockCenter()

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
        # sets new interim arm Pose, now code may proceed as usual 
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
        pixelToMeter = 0.000264583
        xPixelOffSet = cX - self.idealBlockCx
        yPixelOffSet = cY - self.idealBlockCy

        xAxisOffset = pixelToMeter*xPixelOffSet 
        yAxisOffset = pixelToMeter*yPixelOffSet

        pose = self.gripperLimb.endpoint_pose() 
        newPos = [pose['position'].x, pose['position'].y, pose['position'].z]
        quat =  [pose['orientation'].x,pose['orientation'].y,
                    pose['orientation'].z,pose['orientation'].w]
        #newPos to align gripper with block 
        newPos = [pose['position'].x + xAxisOffset, 
                  pose['position'].y - yAxisOffset, 
                  pose['position'].z
                  ]
        jointSolution = self.IKsolver(newPos, quat, self.limb)
        self.moveGripper(jointSolution)
        return 


    def moveGripperYposition(self, grabStack, grabStackHeight):
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
        pose = Locomotion.interimPose
        pos, quat = self.modifyReachPose(placeStack, placeStackHeight, pose)
        jointSolution = self.IKsolver(pos, quat, self.limb)
        self.moveGripper(jointSolution)
        self.handleBlock(False) #drop block 
        return 


    def readInstructions(self, move, bypass=False,depth=0):
        grabStack, grabStackHeight, placeStack, placeStackHeight = move

        if not bypass:
        #if gripper realigned don't move to default poisition  
            self.moveGripperYposition(grabStack, grabStackHeight)

        blockCx, blockCy = self.getBlockCenter() #get center of block 
        if depth == 5:
            blockCx, blockCy = self.idealBlockCx, self.idealBlockCy

        if blockCx in self.blockCxRange and blockCy in self.blockCyRange:
            #move gripper on z-axis to grab block 
            self.moveToGrabBlock(grabStack, grabStackHeight)
            
            self.moveGripper(Locomotion.interimJointAngles) 
            #move gripper to interim position 

            self.moveToPlaceBlock(placeStack, placeStackHeight)
        
            self.moveGripper(Locomotion.interimJointAngles)
            #reset to interim position
        else:
            #use IK solver for adjustment as opposed to just going to next stack
            self.realignGripper(blockCx, blockCy)
            self.readInstructions(move, bypass=True, depth=depth+1) 
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
            time.sleep(.2)
        return 
    
    def moveGripper(self, jointSolution):
        #takes in instructions from IK solver and moves arm 
        #blocking, use set_joint_position to interrupt (unreliable)
        threshold = 0.005726646
        self.gripperLimb.move_to_joint_positions(jointSolution, threshold=threshold)
        return 


# #############################################
#BackEnd Data Manipulation
########################################

def addTowersToGameDict(data):
    #add additional towers to game dictionary 
    for tempTowerNum in range(data.tempTowerCount):
        towerName = 'Temp%d' % (tempTowerNum + 1)
        data.gameDict[towerName] = []

def addBlockToGameDict(data, block, tower):
    #pop/insert blocks into GameDict
    if tower.tower == 0: 
        tower = 'Source'
    elif tower.tower == len(data.towerBases) - 2: 
        tower = 'Target'
    elif tower.tower == len(data.towerBases) - 1: 
        tower = 'UberTemp'
    else: 
        tower = 'Temp%d' % tower.tower 
    data.gameDict[tower].append(block)
    return 

def modifyGameDict(data, towerNum, pop, block=None):
    towerName = getTowerName(data, towerNum)
    if pop: #pop block out of its old tower
        data.gameDict[towerName].pop(-1)
    else: #insert block into its new tower
        data.gameDict[towerName].insert(len(data.gameDict[towerName]), block)
    return 

###################
#Helpful Functions
###################

def rgbString(red, green, blue):
    return "#%02x%02x%02x" % (red, green, blue)

def smoothSpeed(data, start, end):
    dist = abs(end - start)
    delta = data.delta
    if end >= start: plus = True
    else: plus = False
    velocities = [] #list containgin how x or y changes for movement
    while(dist > 0):
        if(dist - delta) >= 0: 
            dist -= delta
            if plus: velocities += [delta]
            elif not plus: velocities += [-delta]
        else: 
            delta -= 1
    return velocities

def getTowerNumber(data, towerName):
    if towerName == 'Source':
        return 0
    elif towerName == 'Target':
        return len(data.towerBases) - 2
    elif towerName == 'UberTemp':
        return len(data.towerBases) - 1
    else: 
        return int(towerName[::-1][0]) #temp tower number
    
def getTowerName(data, towerNum):
    if towerNum == 0: 
        return 'Source'
    elif towerNum == len(data.towerBases) - 2:
        return 'Target'
    elif towerNum == len(data.towerBases) - 1: 
        return 'UberTemp'
    else: 
        return 'Temp%d' % towerNum

def getStackHeight(data, towerNum):
    #print(len(data.towerBases))
    blockHeights = 0
    if towerNum == 0: 
        tower = 'Source'
    elif towerNum == len(data.towerBases) - 2: 
        tower = 'Target'
    elif towerNum == len(data.towerBases) - 1:
        tower = 'UberTemp'
    else: 
        tower = 'Temp%s' % str(towerNum)

    for block in data.gameDict[tower]:
        blockHeights += 2*block.w 
    return blockHeights

#######################
#Classes
#######################

class Block(object):
    blockCount = 0

    def __init__(self, x, y, width, num):
        Block.blockCount += 1
        self.x = x
        self.y = y
        self.w = width
        self.blockFill = 'gray'
        self.textFill = 'black'
        self.blockNum = num
        # self.blockX1, self.blockX2 = self.x - self.w, self.x + self.w
        # self.blockY1, self.blockY2 = self.y - self.w, self.y + self.w

    def __eq__(self, other):
        return (isinstance(other, Block) and self.blockNum == other.blockNum)


    def containsBlock(self, x, y):
        return((x > self.blockX1 and x < self.blockX2) and 
                (y > self.blockY1 and y < self.blockY2))


    def draw(self, canvas, data):
        self.blockX1, self.blockX2 = self.x - self.w, self.x + self.w
        self.blockY1, self.blockY2 = self.y - self.w, self.y + self.w
        canvas.create_rectangle(self.blockX1, self.blockY1, self.blockX2, self.blockY2, 
                                 fill=self.blockFill, activefill='yellow')
        
        canvas.create_text(self.blockX1 + self.w, self.blockY1 + self.w, 
                        text=str(self.blockNum))

    def blockSnapDown(self, data, x, y):
    #snap block down on to the nearest tower when user released right click hold
        for tower in data.towerBases:
            towerBase = data.height - data.floor
            towerX1 = tower.x - data.towerBaseWidth
            towerX2 = tower.x + data.towerBaseWidth

            towerY1 = (towerBase - getStackHeight(data, tower.tower) 
                                                    - 2*data.currentBlock.w)
            towerY2 = towerY1 + 2*data.currentBlock.w
            if((x > towerX1 and x < towerX2) and
                (y > towerY1 and y < towerY2) and 
                self.isLegalPlacement(data, tower)):
                data.currentBlock.x = tower.x 
                blockY = (tower.y -getStackHeight(data,tower.tower) 
                                            - data.currentBlock.w)
                data.currentBlock.y = blockY
                addBlockToGameDict(data, data.currentBlock, tower)
                data.gameDict['Placed'].append(data.currentBlock)

    
    def isLegalPlacement(self, data, tower):
        towerNum = tower.tower
        towerName = getTowerName(data, towerNum)
        tower = data.gameDict[towerName]
        if len(tower) == 0 and towerName != 'UberTemp':
            data.legalMove = True
            return True
        elif len(tower) > 0:
            occupyingBlockNum = tower[-1].blockNum
            if occupyingBlockNum > data.currentBlock.blockNum:
                data.legalMove = True
                return True
        data.legalMove = False
        return False

class ValueSelect(object):
    width = 80
    height = 40

    def __init__(self, x, y, function):
        #function pertains to what value the widget will be manipulating
        self.x = x
        self.y = y
        self.h = ValueSelect.height
        self.w = ValueSelect.width
        self.clickCount = 0
        self.function = function

    def draw(self, canvas):
        #left button
        height = self.y + self.h
        canvas.create_rectangle(self.x, self.y, self.x+self.w/3, height,
                                fill='grey', activefill='yellow')
        canvas.create_text(self.x+self.w/6, self.y+self.h/2, text='+', 
                            fill='black')
        #value viewer
        centerX1 = self.x+self.w/3
        canvas.create_rectangle(centerX1, self.y, centerX1 + self.w/3, height)
        canvas.create_text(centerX1 + self.w/6, self.y + self.h/2, 
                            text=str(self.clickCount), fill='black')

        #right button
        rightX1 = centerX1 + self.w/3
        canvas.create_rectangle(rightX1, self.y, rightX1 + self.w/3, height, 
                                fill='grey', activefill='yellow')
        canvas.create_text(rightX1 + self.w/6, self.y + self.h/2, text='-', 
                            fill='black')

    def containsSelector(self, x, y):
        xIncrementMin = self.x
        xInrementMax = xIncrementMin + self.w/3

        x1Decrem = self.x+self.w*2/3
        xDecrementMin = x1Decrem
        xDecrementMax = xDecrementMin + self.w/3

        yMin = self.y
        yMax = yMin + self.h

        if (y >= yMin) and (y <= yMax):
            if (x >= xIncrementMin) and (x <= xInrementMax): 
                return 'increment'
            elif (x >= xDecrementMin) and (x <= xDecrementMax): 
                return 'decrement'
        return False


class Claw(object):

    barWidth = 8
    barMargin = 5

    def __init__(self, x, y):
        #x, y represent line of symmetry along x-axis through Claw
        self.x = x 
        self.y = y
        self.extension = 5 #min/default extension val
        self.clawUnitColor = rgbString(17, 82, 52)
        self.cylinUnitColor = rgbString(142, 149, 135)
        self.gripperColor = rgbString(112, 134, 124)
        self.mainUnitColor = rgbString(134, 159, 109)

    @staticmethod
    def drawRails(canvas, data):
        #draws 2 bars claw will move along
        barWidth = Claw.barWidth
        barMargin = Claw.barMargin
        barX1,barX2 = data.margin, data.width - data.margin
        topBarY1 = data.margin + barMargin
        topBarY2 = topBarY1 + barWidth
        canvas.create_rectangle(barX1, topBarY1, barX2, topBarY2, 
                                fill='dark grey')
        bottomBarY1 = topBarY2 + barMargin
        bottomBarY2 = bottomBarY1 + barWidth
        canvas.create_rectangle(barX1, bottomBarY1, barX2, bottomBarY2,
                                fill='dark grey')

    @staticmethod
    def grab(data, tower, focus):
        tower = data.towerBases[tower]
        if focus == 'x':
            velocities = smoothSpeed(data, data.clawX, tower.x)
        elif focus == 'y': #extend claw cylinder
            velocities = smoothSpeed(data, data.blockClawContact, data.height - data.floor - getStackHeight(data, tower.tower))
        elif focus == 'grip': #changeGripperSpacing
            blockWidth = data.gameDict[tower.name][-1].w
            velocities = smoothSpeed(data, blockWidth, data.gripSpacing - data.gripperWidth)
        elif focus == 'gripReach':
            blockWidth = data.gameDict[tower.name][-1].w
            velocities = smoothSpeed(data, data.gripperHeight, blockWidth*2)
            data.grabbedBlock = data.gameDict[tower.name][-1] 
            #allows for linkage between block and claw
        return velocities

    @staticmethod
    def move(data, startTower, endTower, focus):
        #raise claw and move it along x-axis to hover above desired tower
        block = data.grabbedBlock
        block.x = data.clawX 
        block.y = data.blockClawContact + block.w
        #bind x and y attributest of block to claw so they move as one

        startTower = data.towerBases[startTower].x
        endTower = data.towerBases[endTower].x

        if focus == 'x':
            velocities = smoothSpeed(data, startTower, endTower)
        elif focus == 'y':
            velocities = smoothSpeed(data, data.cylinderExtension, 0)
        return velocities

    @staticmethod
    def drop(data, endTower, focus):
        data.grabbedBlock.x = data.clawX
        data.grabbedBlock.y = data.blockClawContact + data.grabbedBlock.w
        tower = data.towerBases[endTower]
        if focus == 'y':
            dist = data.height - data.floor - getStackHeight(data, tower.tower) - 2*data.grabbedBlock.w
            velocities = smoothSpeed(data, data.blockClawContact, dist)
        return velocities

    @staticmethod
    def resetClaw(data):
        data.grabbedBlock = None
        velocities = smoothSpeed(data, data.blockClawContact, 60)
        return velocities

    def draw(self, canvas, data):
        x = data.clawX
        y = data.clawY
        if len(data.xVelocities) != 0:
            data.clawX += data.xVelocities.pop(0)
        if len(data.yVelocities) != 0:
            data.cylinderExtension += data.yVelocities.pop(0)
        if len(data.gripVelocities) != 0:
            data.gripSpacing -= data.gripVelocities.pop(0)
        if len(data.gripperHeightVelocities) != 0:
            data.gripperHeight += data.gripperHeightVelocities.pop(0)

        mainUnitWidth = 40
        mainUnitY1offset, mainUnitY2offset = 2, 15
        mainUnitX1, mainUnitX2 = x - mainUnitWidth, x + mainUnitWidth
        mainUnitY1 = data.margin + Claw.barMargin - mainUnitY1offset
        mainUnitY2 = mainUnitY1 + Claw.barWidth*2 + Claw.barMargin + mainUnitY2offset
        canvas.create_rectangle(mainUnitX1, mainUnitY1, mainUnitX2, mainUnitY2, 
                                fill=self.mainUnitColor)
        #draw cylinder that will exted when claw grabs block
        cylinUnitWidth = 20 #really half the width
        cylinUnitHeight = self.extension + data.cylinderExtension
        cylinUnitX1 = x - cylinUnitWidth
        cylinUnitX2 =  x + cylinUnitWidth
        cylinUnitY1 = mainUnitY2
        cylinUnitY2 = cylinUnitY1 + cylinUnitHeight
        canvas.create_rectangle(cylinUnitX1, cylinUnitY1, cylinUnitX2, cylinUnitY2, 
                                fill=self.cylinUnitColor)

        #draws piece that connects cylinder to grippers on claw
        clawUnitWidth = data.gripSpacing #really half the width
        clawUnitHeight = 10
        clawUnitX1 = x - clawUnitWidth
        clawUnitX2 = x + clawUnitWidth
        clawUnitY1 = cylinUnitY2
        clawUnitY2 = clawUnitY1 + clawUnitHeight
        data.blockClawContact = clawUnitY2 #where block becomes flush with claw
        canvas.create_rectangle(clawUnitX1, clawUnitY1, clawUnitX2, clawUnitY2, 
                                fill=self.clawUnitColor)

        #draw left gripper
        lGripperX1 = clawUnitX1
        lGripperX2 = lGripperX1 + data.gripperWidth
        lGripperY1 = clawUnitY2
        lGripperY2 = lGripperY1 + data.gripperHeight
        canvas.create_rectangle(lGripperX1, lGripperY1, lGripperX2, lGripperY2, 
                                fill=self.gripperColor)
        #draw right gripper
        rGripperX1 = clawUnitX2 - data.gripperWidth
        rGripperX2 = clawUnitX2
        rGripperY1, rGripperY2 = clawUnitY2, clawUnitY2 + data.gripperHeight
        canvas.create_rectangle(rGripperX1, rGripperY1, rGripperX2, rGripperY2, 
                                fill=self.gripperColor)

class TowerBase(object):

    def __init__(self, x, y, tower):
        self.x = x
        self.y = y
        self.baseColor = rgbString(208, 166, 51)
        self.tower = tower

    def draw(self, canvas, data):
        #draw base tower will be stacked on 
        towerWidth, towerHeight = data.towerBaseWidth, data.towerBaseHeight
        baseX1, baseX2 = self.x - towerWidth, self.x + towerWidth
        baseY1 = self.y 
        baseY2 = baseY1 + towerHeight
        canvas.create_rectangle(baseX1, baseY1, baseX2, baseY2, 
                                fill=self.baseColor)
        self.name = getTowerName(data, self.tower)

        canvas.create_text(baseX1 + towerWidth, baseY1 + towerHeight/2, 
                            text=self.name)

    @staticmethod    
    def initializeTowerBases(data):
        #initialize tower bases 
        totalTowers = data.tempTowerCount + 3 #Source, Target, UberTemp
        baseOffset = 100 #prevents tower base from going off screen
        data.towerSpacing = data.width/(totalTowers + 1)
        for tower in range(totalTowers):
            data.towerBases.append(TowerBase(tower*data.towerSpacing + 
                        data.towerSpacing, data.height-data.floor, tower))
        return 

class FollowInstructions(object):
    def __init__(self, instructions=None):
        self.moves = instructions

    def convertMove(self):
        pass

    @staticmethod
    def clawGrabCallback(data, tower1=None):
        if data.clearForGrab: #grab block
            tower1 = getTowerNumber(data, tower1)
            data.xVelocities = Claw.grab(data, tower1, 'x') #move to target tower
            if len(data.xVelocities) == 0:
                data.yVelocities = Claw.grab(data, tower1, 'y') #extend to block
                if len(data.yVelocities) == 0:
                    data.gripVelocities = Claw.grab(data, tower1, 'grip') #wrap around block
                    data.gripperHeightVelocities = Claw.grab(data, tower1, 'gripReach') #extend grippers for firm grab
                    if len(data.gripperHeightVelocities) == 0:
                        data.clearForGrab = False
                        data.clearForMove = True #move to next tower
                        return True 
    @staticmethod
    def clawMoveCallback(data, tower1=None, tower2=None):
        if data.clearForMove: #move block to new tower
            tower1 = getTowerNumber(data, tower1)
            tower2 = getTowerNumber(data, tower2)
            data.yVelocities = Claw.move(data, tower1, tower2, 'y') #retract gripper
            if len(data.yVelocities) == 0:
                data.xVelocities = Claw.grab(data, tower2, 'x') #move to target tower
                if len(data.xVelocities) == 0: #reached target tower
                    data.clearForMove = False
                    modifyGameDict(data, tower1, pop=True) #remove block from old tower
                    data.clearForDrop = True #drop block
    
    @staticmethod
    def clawDropCallback(data, tower2=None):
        if data.clearForDrop: #place block on new tower
            tower2 = getTowerNumber(data, tower2)
            data.yVelocities = Claw.drop(data, tower2, 'y') #extend cylinder for drop
            if len(data.yVelocities) == 0:
                data.clearForDrop = False
                modifyGameDict(data, tower2, pop=False, block=data.grabbedBlock)
                #insert block into its new tower 
                data.resetClaw = True 
        
    @staticmethod
    def clawResetCallback(data):
        if data.resetClaw: #get ready for next instruction
            data.grabbedBlock = None
            data.yVelocities = Claw.resetClaw(data) 
            if len(data.yVelocities) == 0:
                if len(data.moves) != 0:
                    data.clearForGrab = True
                    data.moves.pop(0)
                    data.grabbedBlock = None
                    data.resetClaw = False
                else: 
                    data.solutionFound = False
    @staticmethod
    def convertGameDict(data):
        #convert gameDict to backEndDict for unscrambling algorithm 
        for tower in data.gameDict:
            if tower == 'Placed':
                continue
            data.backEndDict[tower] = []
        for tower in data.gameDict:
            if tower == 'Placed':
                continue
            for block in data.gameDict[tower]:
                data.backEndDict[tower].insert(0, block.blockNum)

    @staticmethod
    def findSolution(data):
        solve = solveTowers()
        data.moves = solve.scrambledHanoi(data.backEndDict, len(data.blocks))

class solveTowers(object):

    def __init__(self):
        self.moves = []
   

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

    def hanoi(self, stack, source, target, temp):
        #wrapper for solver 
        self.move(stack, source, target, temp)

    def move(self, stack, source, target, temp):
        
        if stack == 1:
            self.moves += [(source, target)]
            # add to growing instructions list 
        else:
            self.move(stack-1, source, temp, target)
            self.move(1, source, target, temp)
            self.move(stack-1, temp, target, source)

    def findSmallestTower(self, setup):
        smallestTower, smallestStack = None, None

        for tower in setup.keys():
            #print(tower)
            if tower == 'SortedStack': # or tower == 'Temp5': 
                continue # prevent vals from being popped from sortedStack
            currentStack = len(setup[tower])
            if smallestStack == None: 
                smallestStack = currentStack
                smallestTower = tower
            if currentStack < smallestStack:
                smallestTower = tower
                smallestStack = currentStack
        print(smallestTower)
        return smallestTower

    def findBlock(self, setup, block):
        for tower in setup.keys():
            if tower == 'SortedStack': 
                continue #prevent vals from being popped from sortedStack
            if block in setup[tower]: 
                return tower
        return False #block not found 

    def unscrambleSetup(self, setup, currentBlock, nextBlock):
        target, source = None, None
        # init to None for edge case when nextBlock DNE
        for tower in setup.keys():
            if tower == 'SortedStack': #or tower == 'Temp5': 
                continue # prevent vals from being popped from sortedStack
            if currentBlock in setup[tower]: #find tower of desired blcok
                source = tower
            if nextBlock in setup[tower]: #find tower where desired block goes
                target = tower

        temp = self.findSmallestTower(setup)
        return (source, target, temp)

    def correctTowers(self, setup, totalBlocks):
        # unscrambles towers so normal game of TOH can be played 
        scrambled = True 
        for tower in setup.keys():
            if tower == 'SortedStack': 
                continue # prevent vals from being popped from sortedStack
            if len(setup[tower]) == totalBlocks: #all blocks in one tower
                scrambled = False 
                setup['SortedStack'] = range(0, totalBlocks)
                # ensure that all blocks inside of sortedStack 
                if tower == 'Target': #all blocks in target tower, done
                    return True 
                else: #all blocks in 1 tower, play normal TOH
                    source = tower # tower with all the blocks in it 
                    target = 'Target' 
                    # temp = self.findSmallestTower(setup)
                    temp = 'UberTemp'
        if scrambled: #towers still scrambled 
            currentBlock  = len(setup['SortedStack'])
            setup['SortedStack'] += [currentBlock]
            nextBlock = currentBlock + 1
            source, target, temp = self.unscrambleSetup(setup, currentBlock, nextBlock)

        return source, target, temp

    def modifyTowersDict(self, setup, source, target):
        newSourceTower = list(set(setup[source]) - set(setup['SortedStack']))
        newTargetTower = setup['SortedStack'] + setup[target]
        if len(newSourceTower) < 2:
            setup[source] = newSourceTower
        else:
            setup[source] = sorted(newSourceTower)
        #pop vals in sortedStack out of source
        setup[target] = newTargetTower
        #add vals from sortedStack to the target tower
        return setup

    def scrambledHanoi(self, setup, totalBlocks): 
        while(len(setup['Target']) != totalBlocks): 
            source, target, temp = self.correctTowers(setup, totalBlocks)
            stack = len(setup['SortedStack'])

            if source == target: #work done for me
                nextBlock = setup[source][-1] #last block in tower
                nextTower = self.findBlock(setup, nextBlock+1)
                setup[nextTower] = setup[source] + setup[nextTower]
                self.hanoi(len(setup[source]), source, nextTower, 'UberTemp')
                nextVal = setup['SortedStack'][-1] + 1 
                setup[source] = []
                setup['SortedStack'] += [nextVal]
            else:
                self.hanoi(stack, source, target, temp) #stores moves in self.moves list
                setup = self.modifyTowersDict(setup, source, target)
        return self.moves #fingers crossed


##########################
#Splash Screen Drawings
##########################

def drawSplashScreen(canvas, data): #mode = 'splashScreen'
    topText = 'Welcome to ROBO'
    middleText = 'Towers of Hanoi'
    MyCreds = 'A Game by Obi Adubor'
    bottomText = 'Press Any Key to Continue'
    canvas.create_rectangle(data.width//2, data.height//2, (data.width//2) + 10, data.height, fill='red')
    #image = data.images['homeImage']
    #canvas.create_image(data.width/2, data.height/2,  image=image)

    canvas.create_text(data.width/4, data.height/2, 
                        fill='black', text=topText, 
                        font="Helvetica 40")
    canvas.create_text(data.width/4, data.height/1.7, 
                        fill='black', text=middleText, 
                        font="Helvetica 40")
    canvas.create_text(data.width/4, data.height/1.5, fill='black', 
                        text = MyCreds, font='Times')
    canvas.create_text(data.width/4, data.height/1.3, 
                        fill='black', text=bottomText, 
                        font="Helvetica 15")
    
def drawInstructionsPage(canvas, data): #mode = 'instructSreen'
    selectorOffset = 30
    gameInstructs1 = """ How to Play: left click to spawn block, hold right click (scroll button on desktop) 
    on block to move it to desired tower, release to place block. 
    When finished press SpaceBar to watch the robot solve your configuration. 
    """
    gameInstructs2 = """ 
   
    Disclaimer:  No blocks can be placed in the 'UberTemp' tower
    and a block is permanently set once placed (press r to restart), you cannot
    modify tower or block number once you move to game screen 
    """
    modeInstructs1 = 'press backspace to return to return home'
    modeInstructs2 = 'press spacebar to go to game'
    modeInstructs3 = 'Please select number of blocks'
    modeInstructs4 = 'Please select number of towers'
    textOffSet = 200
    canvas.create_text(data.width/2, data.height/4, 
                        fill='black', text=gameInstructs1, 
                        font="Helvetica 18")#, width=700,)
    canvas.create_text(data.width/2, data.height/2.2, fill='black', 
                        text=gameInstructs2, font='Helvetica 18')
    canvas.create_text(data.width*3/4, data.height/1.5, 
                        fill='black', text=modeInstructs1, 
                        font="Helvetica 15")
    canvas.create_text(data.width/4, data.height/1.5, 
                        fill='black', text=modeInstructs2, 
                        font="Helvetica 15")
    canvas.create_text(data.width/4, data.height/1.2, 
                        fill='black', font='Helvetica 12 bold', text=modeInstructs3)
    canvas.create_text((data.width*3)/4, data.height/1.2, 
                        font='Helvetica 12 bold',fill='black', text=modeInstructs4)
 
def drawSoloUserBoard(canvas, data):
    #red rectangle that lies bottom of board 
    #image = data.images['gameImage']
    #canvas.create_image(data.width/2, data.height/2,  image=image)
    canvas.create_rectangle(0, data.height-data.floor, data.width, data.height,fill='dark red')
    if data.gameSolved:
        solvedMsg = 'Towers Of Hanoi Solved!'
        canvas.create_text(data.width/2, data.height/1.5, 
                        fill='red', text=solvedMsg, 
                        font="Helvetica 30")
    if not data.legalMove:
        illegalMsg = 'Illegal Move'
        canvas.create_text(data.width/2, data.height/1.5, 
                        fill='red', text=illegalMsg, 
                        font="Helvetica 30")

def drawWinScreen(canvas, data):
    #image = data.images['instructImage']
    #canvas.create_image(data.width/2, data.height/2,  image=image)
    canvas.create_rectangle(0, data.height-data.floor, data.width, data.height,fill='dark red')
    msg1 = 'You Win! Thanks For Playing'
    msg2 = 'Carpe Diem!'
    canvas.create_text(data.width/5, data.height/2, text=msg1, width = 200, 
                        font='Helvetica 30')
    canvas.create_text(data.width/1.2, data.height/2, text=msg2, width = 300, 
                        font='Helvetica 30')


#######################
#Tkinter Framework
#######################

def init(data):
    data.robotReady = True #set to true to incroporate Baxter Robot 
    data.gameDict = {'Source':[], 'Target':[], 'SortedStack':[], 'Placed':[], 'UberTemp':[]} 
    data.backEndDict = {'Source':[], 'Target':[], 'SortedStack':[], 'UberTemp':[]}
    data.mode = 'splashScreen' 
    data.blocks = []
    data.valueSelectors = []
    data.towerBases = []
    data.xVelocities, data.yVelocities, data.cylinderVelocities = [], [], []
    data.gripVelocities, data.gripperHeightVelocities = [], []
    data.blockWidth, data.deltaBlockWidth = 40, 5
    data.towerBaseWidth, data.towerBaseHeight = 50, 30
    data.maxTempTowers, data.maxBlocks = 5, 6
    data.blockCount = 0 # num of blocks user selected to be available
    data.tempTowerCount = 1
    data.clawX, data.clawY = 100, 15
    data.claw = Claw(data.clawX, data.clawY) 
    data.cylinderExtension = 0 #manipulate to extend gripper to grab block 
    data.gripSpacing = 60 #manipulate to have gripper grab block 
    data.gripperHeight, data.gripperWidth = 40, 7
    data.floor = 50 #where floor appears on userBoard Screen (from bottom)
    data.currentBlock = None #current block double clicked to be moved
    data.grabbedBlock = None
    data.currentMove = None
    data.clearForGrab, data.clearForMove, data.clearForDrop =False, False, False
    data.resetClaw = False
    data.solutionFound = False
    data.gameSolved = False
    data.selectorOffset = 30
    data.towerMod = False #prevent modding of tower after going to game screen
    data.delta = 10 #initial claw speed
    data.moves = [] #where instructions will be stored
    data.movesOldLen = len(data.moves)
    data.legalMove = True #desired block placement is legal move
    # if data.robotReady:
    #     data.cam = gripperCameraFeed() 
    #     data.robotMove = Locomotion() #Initiate Robot Kinematics
    #     data.robotMove.setDefaultAttributes()
    #     data.robotMove.setArmDefaultPosition()
    #     data.robotMove.calibrateGripper()
    data.valueSelectors.append(ValueSelect(data.width/4 - data.selectorOffset,
                                                     data.height/1.1, 'block'))
    #block value means widget will manipulate # of available blocks
    data.valueSelectors.append(ValueSelect(data.width*3/4 - data.selectorOffset,
                                             data.height/1.1, 'tower'))
     #tower value means widget will manipulate # of additional

#def initPhotos(data):
#    data.homeImage = PhotoImage(file="wall-ESpaceStareBackground.ppm")
#    data.images["homeImage"] = data.homeImage
#    data.instructImage = PhotoImage(file='Wall-EHang.ppm')
#    data.images['instructImage'] = data.instructImage
#    data.gameImage = PhotoImage(file='wall-ETrashBackground.ppm')
#    data.images['gameImage'] = data.gameImage

def mouseHold(event, data):
    if data.mode == 'soloUserBoard':
        #if not data.blockSelected:
        for block in data.blocks:
            if block.containsBlock(event.x, event.y) and block not in data.gameDict['Placed']:
                #not allowed to move block position after it's been set 
                if data.currentBlock == None:
                    data.currentBlock = block
                    block.x = event.x 
                    block.y = event.y 
                elif(data.currentBlock == block):# and 
                    block.x = event.x 
                    block.y = event.y 

def mouseRelease(event, data):
    #called when user releases right click
    if data.mode == 'soloUserBoard':
        if data.currentBlock != None:
            data.currentBlock.blockSnapDown(data, event.x, event.y)
            # data.gameDict['Placed'].append(data.currentBlock)
        data.currentBlock = None
        if data.blockCount == len(data.gameDict['Placed']):
            data.clearForGrab = True

def mousePressed(event, data):
    if data.mode == 'soloUserBoard':
        if len(data.blocks) < data.blockCount:
            data.blocks.append(Block(event.x - data.blockWidth/2, 
                                 event.y - data.blockWidth/2, data.blockWidth, 
                                data.blockNum))
            data.blockNum -= 1
            data.blockWidth -= data.deltaBlockWidth


    if data.mode == 'instructScreen':
        for selector in data.valueSelectors:
            selection = selector.containsSelector(event.x, event.y)
            if selection == 'increment': #increment button pressed
                if(selector.function == 'block' and 
                        data.blockCount < data.maxBlocks): 
                    selector.clickCount += 1
                    data.blockCount += 1
                if selector.function == 'tower': #increment tower button pressed
                    if selector.clickCount < data.maxTempTowers:
                        selector.clickCount += 1
                        data.tempTowerCount += 1
            elif selection == 'decrement': #decremenet button pressed 
                if selector.clickCount > 0: 
                    selector.clickCount -= 1
                    if selector.function == 'tower': data.tempTowerCount -= 1
                    elif selector.function == 'block': data.blockCount -= 1

def keyPressed(event, data):
    if data.mode == 'splashScreen' and event.keysym:
        data.mode = 'instructScreen'
    elif data.mode == 'instructScreen': 
        if event.keysym == 'BackSpace':
            data.mode = 'splashScreen'
        elif event.keysym == 'space':
            data.valueSelectors = [] #can't mess with towers once game starts 
            data.blockNum = data.blockCount - 1 # num block user added to game
            TowerBase.initializeTowerBases(data)
            addTowersToGameDict(data)
            data.mode = 'soloUserBoard'
        elif event.keysym == 'r':
            init(data)

    elif data.mode == 'soloUserBoard':
        if event.keysym == 'BackSpace':
            data.mode = 'instructScreen'
        elif event.keysym == 'space':
            data.solution = FollowInstructions()
            data.solution.convertGameDict(data)
            data.solution.findSolution(data)
            data.solutionFound = True
            data.clearForGrab = True
            print(data.moves)
        elif event.keysym == 'r':
            init(data)
        elif event.keysym == 'w':
            data.mode = 'win'

    elif data.mode == 'win':
        if event.keysym == 'r':
            init(data)
        

def sendToRobot(data, tower1, tower2):
    if data.movesOldLen != len(data.moves): 
        data.movesOldLen = len(data.moves)
        tower1Height = len(data.gameDict[tower1])
        tower2Height = len(data.gameDict[tower2])
        tower1 = getTowerNumber(data, tower1)
        tower2 = getTowerNumber(data, tower2)
        robotMove = [tower1, tower1Height, tower2, tower2Height]
        data.robotMove.readInstructions(robotMove)

def timerFired(data):

    if data.moves == None:
        data.gameSolved = True 
        data.mode = 'win'

    elif data.solutionFound and len(data.moves) > 0:
        #data.movesOldLen = len(data.moves)
        data.currentMove = data.moves[0]
        tower1, tower2 = data.currentMove
        data.tower1, data.tower2 = tower1, tower2
        if data.robotReady:
            sendToRobot(data, tower1, tower2) #send move to robot
        data.solution.clawGrabCallback(data, tower1)
        #grab the Block
        data.solution.clawMoveCallback(data, tower1, tower2) 
        #move block to new tower
        data.solution.clawDropCallback(data, tower2) 
        #place block on new tower
        data.solution.clawResetCallback(data) 
        if len(data.moves) == 0: #game over
            data.moves = None


def redrawAll(canvas, data):
    #draw border
    canvas.create_rectangle(data.margin, data.margin, data.width, data.height, 
        width=data.margin)
    if data.mode == 'splashScreen': 
        drawSplashScreen(canvas, data)
    elif data.mode == 'instructScreen': 
        drawInstructionsPage(canvas, data)
        for selector in data.valueSelectors:
            selector.draw(canvas)
    elif data.mode == 'soloUserBoard': 
        drawSoloUserBoard(canvas, data)
        for block in data.blocks:
            block.draw(canvas, data) #drawBlocks
        for base in data.towerBases: 
            base.draw(canvas, data) #draw towerBases
        Claw.drawRails(canvas, data) #draw rails for claw 
        data.claw.draw(canvas, data) #draw claw
    elif data.mode == 'win':
        drawWinScreen(canvas, data)


def run(width=1000, height=500):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        redrawAll(canvas, data)
        canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def mouseHoldWrapper(event, canvas, data):
        mouseHold(event, data)
        redrawAllWrapper(canvas, data)

    def mouseReleaseWrapper(event, canvas, data):
        mouseRelease(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Struct(object): pass
    data = Struct()
    data.width = width
    data.height = height
    data.margin = 4
    data.timerDelay = 50 # milliseconds
    data.images = {} #stores screen images 
    init(data)
    # create the root and the canvas
    root = Tk()
    #canvas.create_rectangle(data.width//2, data.height//2, (data.width//2) + 10, data.height, fill='red')
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.pack()
    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<B2-Motion>", lambda event:
                            mouseHoldWrapper(event, canvas, data))
    root.bind("<ButtonRelease-2>", lambda event:
                            mouseReleaseWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed

####################################
# playScrambledToh() [calls run()]
####################################

def playScrambledTOH():
    run()



def main(args):
    rospy.init_node("gripperCameraFeed", anonymous=True)
    playScrambledTOH()
    try:    
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
