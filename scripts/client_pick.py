#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
import struct
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib import SimpleActionClient
import tf
import baxter_interface
import numpy as np
from std_srvs.srv import Empty
from math import pi 
import cv2
from cv_bridge import CvBridge

from lab_ros_perception.AprilTagModule import AprilTagModule
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class TagsPose(object):
    def __init__(self):
        rospy.loginfo("Initalizing...")
        
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        time.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pickup_pose AS")
            exit()
            
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)
        self.place_as.wait_for_server()

        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing TagsPose.")
        self.poses ={}
        self.limb = baxter_interface.Limb('right')
        self.tag_module = AprilTagModule()
        self.transformedDict = {}
        self.jointAngles = {}
        self.t = tf.TransformListener()
        frameId = 'camera_rgb_optical_frame'
        self.t.waitForTransform('base', frameId,rospy.Time(), rospy.Duration(1))
        time.sleep(1)
        
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        self.orien_trash = []
        self.size = []
        
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
        self.newPose.pose.position.x +=0.025
        self.newPose.pose.position.y +=0.07
        self.newPose.pose.position.z -= 0.01
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
        return self.transformedDict , self.jointAngles 
        
        
    def addTrashAsObstacles(self):
        self.trashposes,self.baxjoints =self.makeDictofTransformedPoses()
        for key, val in self.trashposes.items():
            val = self.trashposes[key]
            self.trashloc = val.pose
            self.trash_loc_x.append(self.trashloc.position.x)
            self.trash_loc_y.append(self.trashloc.position.y)
            self.trash_loc_z.append(self.trashloc.position.z)
            self.orien_trash.append(self.trashloc.position.z*pi/180)
            return self.trash_loc_x, self.trash_loc_y, self.trash_loc_z        

        
    def pickandplace_apriltag(self):
        self.trash_loc_x, self.trash_loc_y, self.trash_loc_z = self.addTrashAsObstacles()
        rospy.sleep(2.0)
        rospy.loginfo("spherical_grasp: Waiting for an April Tag detection")
        ps = PickUpPoseGoal()
        ps.object_pose.header.stamp = rospy.Time.now()
        ps.object_pose.header.frame_id = "base"
        ps.object_pose.pose.position.x = self.trash_loc_x[0]
        ps.object_pose.pose.position.y = self.trash_loc_y[0]
        ps.object_pose.pose.position.z = self.trash_loc_z[0]
        ps.object_pose.pose.orientation.w = 1.0
        self.pick_as.send_goal_and_wait(ps)
        result = self.pick_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            rospy.logerr("Failed to pick, not trying further")
            return
        tc = PickUpPoseGoal()
        tc.object_pose.header.stamp = rospy.Time.now()
        tc.object_pose.header.frame_id = "base"
        tc.object_pose.pose.position.x = 1.03
        tc.object_pose.pose.position.y = -0.415
        tc.object_pose.pose.position.z = 0.01
        tc.object_pose.pose.orientation.w = 1.0
        self.place_as.send_goal_and_wait(tc)
        rospy.loginfo("Done!")



if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    sphere =TagsPose()
    sphere.pickandplace_apriltag()
#    rospy.spin()

