#!/usr/bin/env python
import sys
import os
import struct
import numpy as np
import rospy
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule
from apriltags_ros.msg import AprilTagDetectionArray
import time
import tf
import math
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
        self.newPose.pose.position.z -= 0.02
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
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
#            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
#                  (seed_str,))
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
        self.gripperOpen = True 
        return

    def defineJointAngles(self, positionList):
        self.angles['right_s0']= positionList[0]
        self.angles['right_s1']= positionList[1]
        self.angles['right_e0']= positionList[2]
        self.angles['right_e1']= positionList[3]
        self.angles['right_w0']= positionList[4]
        self.angles['right_w1']= positionList[5]
        self.angles['right_w2']= positionList[6] 

    def moveArm(self):
#        self.newPosesDict, self.joints = TagsPose.makeDictofTransformedPoses()
#        for key, val in self.joints.items():
        pass



def main(args):
    rospy.init_node("TagsPose", anonymous=True)
    ic = TagsPose()
#    x = ic.getDictofPoses()
#    x = ic.transform_pose()
    x = ic.makeDictofTransformedPoses()
    print x
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)