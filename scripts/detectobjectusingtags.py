#!/usr/bin/env python
import sys
import os
import struct
import numpy as np
import rospy
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule
from apriltags_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, PoseArray
import time
import tf
from moveit_python import PlanningSceneInterface
from math import pi 
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)




class TagsPose(object):
    def __init__(self):
        self.poses ={}
        self.limb = baxter_interface.Limb('right')
        self.tag_module = AprilTagModule()
        self.transformedDict = {}
        self.jointAngles = {}
        self.t = tf.TransformListener()
        frameId = 'camera_rgb_optical_frame'
        self.t.waitForTransform('base', frameId,rospy.Time(), rospy.Duration(1))
        time.sleep(1)
        
        
    def getDictofPoses(self):
        while True:
            """
            set stamped = 1 to return the LAST stamped pose of all seen IDs 
            set stamped = 0 to return the updated version of the pose (it refreshes every 5 seconds). 
            """
            self.poses = self.tag_module.getPosesForSeenIDs(stamped =0)
            if self.poses != {}:
                return self.poses 
            
            
    def transform_pose(self, ids,t):
        self.newPose= t.transformPose('base',ids)
        # change the orientation (quaternions) of april tags so that the IK can work
        # need to change it so Baxter knows were to grab the tags from
        self.newPose.pose.position.z += 0.20
        self.newPose.pose.orientation.x = 0
        self.newPose.pose.orientation.y = 1.0
        self.newPose.pose.orientation.z = 0
        self.newPose.pose.orientation.w = 0
        return self.newPose


    def checkValidPose(self,pose):
        ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's st.waitForTransform('base', frameId,time2, rospy.Duration(1))tring representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
#            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
            # Format solution into Limb API-compatible dictionary
#            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#            print "\nIK Joint Solution:\n", limb_joints
#            print "------------------"
#            print "Response Message:\n", resp
            return resp.joints[0].position 
        else:
#            print("INVALID POSE - No Valid Joint Solution Found.")
            return 0


    def makeDictofTransformedPoses(self):
        dictOfOldPoses = self.getDictofPoses()
        self.transformedDict = {}
        self.jointAngles = {}
        for key, val in dictOfOldPoses.items():
            transformed = self.transform_pose(val,self.t)
            validTransformed = self.checkValidPose(transformed)
            if validTransformed != 0:
                self.jointAngles[key] = validTransformed
                self.transformedDict[key] = transformed
                print self.transformedDict
        return self.transformedDict , self.jointAngles 
    
        


class MoveBaxter(object):
    def __init__(self):
        self.limb = baxter_interface.Limb('right')
        self.gripper = baxter_interface.Gripper('right')
        self.angles = self.limb.joint_angles()

    def calibrateGripper(self):
        #calibrate gripper and close it 
        self.gripper.calibrate(block=True, timeout=2.0)
        self.gripper.command_position(position=100.0, block =True, timeout=5.0)
        return

    def openGripper(self): 
        #opens the gripper
        self.gripper.command_position(position=100.0, block =True,timeout=5.0)
        self.gripper.open() 
        return
    
    def closeGripper(self):
        #closes the gripper
        self.gripper.command_position(position=100.0, block =True,timeout=5.0)
        self.gripper.close()
        

    def defineJointAngles(self, positionList):
        self.angles['right_s0']= positionList[0]
        self.angles['right_s1']= positionList[1]
        self.angles['right_e0']= positionList[2]
        self.angles['right_e1']= positionList[3]
        self.angles['right_w0']= positionList[4]
        self.angles['right_w1']= positionList[5]
        self.angles['right_w2']= positionList[6]
        return self.angles

    def bringArmDown(self, tallpose):
        tallpose.pose.position.z -= 0.20
        tallpose.pose.position.x +=0.02
        return tallpose
    
    def anglesForTrashcan(self, angles):
        angles['right_s0']= 0.516
        angles['right_s1']= -0.661
        angles['right_e0']= -0.816
        angles['right_e1']= 1.583
        angles['right_w0']= 1.311
        angles['right_w1']= 0.548
        angles['right_w2']= -0.942
        return angles 
        
        
    def moveArm(self):
        self.calibrateGripper()
        self.openGripper()
        self.newPosesDict, self.joints = TagsPose.makeDictofTransformedPoses(TagsPose())
        for key, val in self.joints.items():
            angles = self.defineJointAngles(val)
            self.limb.move_to_joint_positions(angles)
#            break
        for key, val in self.newPosesDict.items():
            down = self.bringArmDown(val)
            validTransformed = TagsPose.checkValidPose(TagsPose(),down)
            downangles = self.defineJointAngles(validTransformed)
            self.limb.move_to_joint_positions(downangles)
#            break
            self.closeGripper()
            trashcan = self.anglesForTrashcan(downangles)
            self.limb.move_to_joint_positions(trashcan)
            self.openGripper()
            

class BaxterRangeSensor():
    def __init__(self):
        self.distance = {}
        root_name = "/robot/range/"
        sensor_name = "right_hand_range/state"
        self._left_sensor = rospy.Subscriber(root_name + sensor_name, Range, callback = self._sensorCallback, callback_args = "right", queue_size = 1)
        
    def _sensorCallback(self,msg):
        self.distance["right"] = msg.range
        
class SceneObstacles():
    def __init__(self):
        self.psi = PlanningSceneInterface("base")
        self.psi.clear()
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        self.size = []
        self.orien_trash = []
        
    def addTable(self):
        # attachBox (self, "name", sizex, sizey,sizez, x, y, z, wait = True)
        self.psi.attachBox(self, 'table', 0.75, 0.152, 0.73, 0.75, -0.74, -0.93, wait=True)
        
    def addTrashAsObstacles(self):
        self.trashposes,self.baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
        self.trashloc= self.trashposes.poses
        for i in xrange(len(self.trashloc)):
            self.trash_loc_x.append(self.trashloc[i].position.x)
            self.trash_loc_y.append(self.trashloc[i].position.y)
            self.orien_trash.append(self.trashloc[i].position.z*pi/180)
            self.size.append(self.trashloc[i].position.x)
        while self.trash_loc_x:
            self.psi.clear()
            self.psi.attachBox(self, 'table', 0.75, 0.152, 0.73, 0.75, -0.74, -0.93,'base', touch_links=['pedestal'])
            self.objectlist =['box1','box2', 'box3']
            for i in xrange(len(self.trash_loc_x)):
                self.psi.addCube(self, self.objectlist[i], 0.045,self.trash_loc_x[i], self.trash_loc_y[i],self.trash_loc_z[i])
        self.psi.waitForSync()

    def addTrashcan(self):
        self.tc = PoseStamped()
        self.tc.header.frame_id = "camera_rgb_optical_frame"
        self.tc.header.stamp = rospy.Time.now()
        self.tc.pose.position.x = 0.65
        self.tc.pose.position.y = 0.55
        self.tc.pose.position.z = 0.1
        self.tc.orientation.x = 0
        self.tc.orientation.y = 1
        self.tc.orientation.z = 0
        self.tc.orientation.w =0
        
        
        
        
    


def main(args):
    rospy.init_node("TagsPose", anonymous=True)
#    ic = TagsPose()
    ic = MoveBaxter()
#    x = ic.getDictofPoses()
#    x = ic.transform_pose()
#    x = ic.makeDictofTransformedPoses()
    x = ic.moveArm()
    print x
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)