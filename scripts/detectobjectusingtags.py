#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('baxter_failure')
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

    def getDictofPoses(self):
        tag_module = AprilTagModule()
        while True:
            """
            set stamped = 1 to return the LAST stamped pose of all seen IDs 
            set stamped = 0 to return the updated version of the pose (it refreshes every 5 seconds). 
            """
            self.poses = tag_module.getPosesForSeenIDs(stamped =0)
            if self.poses != {}:
                return self.poses[0] 
            #for i in self.poses:
            #    if self.poses[i] is not None:
            #        print self.poses[i].pose.position.x
    
    def transform_pose(self):
        self.t = tf.TransformListener()
        rate = rospy.Rate(10.0)
        time2 = rospy.Time()
        self.frameId = 'camera_rgb_optical_frame'
        self.t.waitForTransform('base', self.frameId,time2,rospy.Duration(10))
        (trans, rot) = self.t.lookupTransform('base',self.frameId,time2)
        euler = tf.transformations.euler_from_quaternion(rot)
        source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
        px = trans[0]
        py = trans[1]
        pz = trans[2]
        time.sleep(1)
        self.newPose= self.t.transformPose('base',self.getDictofPoses())
        print self.newPose.pose
        # change the orientation (quaternions) of april tags so that the IK can work
        # need to change it so Baxter knows were to grab the tags from
        self.newPose.pose.position.z -= 0.02
        self.newPose.pose.orientation.x = 0
        self.newPose.pose.orientation.y = 1.0
        self.newPose.pose.orientation.z = 0
        self.newPose.pose.orientation.w = 0
        print self.newPose
        self.positionsForBaxter = MoveBaxter.defineJointAngles(MoveBaxter(), self.newPose)
        print self.positionsForBaxter
        self.IKangles = MoveBaxter.moveArm(MoveBaxter(), self.newPose)
        print self.IKangles
        self.limb.move_to_joint_positions(self.IKangles)

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
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
            return resp.joints[0].position 
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

            return 0


class MoveBaxter(object):
    def __init__(self):
        self.limb = baxter_interface.Limb('right')
        self.gripper = baxter_interface.Gripper('right')
        self.angles = self.limb.joint_angles()
        pass

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

    def defineJointAngles(self, pose):
        self.positionList = TagsPose.checkValidPose(TagsPose(),pose)
        return self.positionList

    def moveArm(self,pose):
        self.positionList = self.defineJointAngles(pose)
        self.angles['right_s0']=self.positionList[0]
        self.angles['right_s1']=self.positionList[1]
        self.angles['right_e0']=self.positionList[2]
        self.angles['right_e1']=self.positionList[3]
        self.angles['right_w0']=self.positionList[4]
        self.angles['right_w1']=self.positionList[5]
        self.angles['right_w2']=self.positionList[6]
        return self.angles



def main(args):
    rospy.init_node("TagsPose", anonymous=True)
    ic = TagsPose()
    #x = ic.getDictofPoses()
    x = ic.transform_pose()
    print x
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)