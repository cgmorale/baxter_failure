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
from geometry_msgs.msg import PoseStamped, PoseArray,Pose
import time
import tf
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from math import pi, radians 
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation



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
        self.newPose.pose.position.x -=0
        self.newPose.pose.position.y +=0
        self.newPose.pose.position.z += 0
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
        self.robot = moveit_commander.RobotCommander()
#        self.psi = moveit_commander.PlanningSceneInterface()
        self.psi = PlanningSceneInterface("base")
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        print self.group.get_current_joint_values()
        self.group.get_planning_frame()
        self.group.get_end_effector_link()
#        self.psi.clear()
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        self.size = []
        self.orien_trash = []
#        self.t = tf.TransformListener()
        
    def addTable(self):
        # attachBox (self, "name", sizex, sizey,sizez, x, y, z, wait = True)
        self.psi.attachBox("table", 0.75, 1.52, 0.73, 0.84, 0.2, -0.55, 'base', 'pedestal', wait=True)

    def addTrashAsObstacles(self):
        self.trashposes,self.baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
        for key, val in self.trashposes.items():
            val = self.trashposes[key]
            self.trashloc = val.pose
            self.trash_loc_x.append(self.trashloc.position.x)
            self.trash_loc_y.append(self.trashloc.position.y)
            self.trash_loc_z.append(self.trashloc.position.z)
            self.orien_trash.append(self.trashloc.position.z*pi/180)
            self.size.append(self.trashloc.position.x)
        while self.trash_loc_x:
            self.psi.clear()
            self.psi.attachBox('table', 0.75, 1.52, 0.73, 0.84, 0.2, -0.55,'base', 'pedestal', wait=True)
            self.psi.attachBox("trashcan",0.365, 0.265,0.39,1.03,-0.415, 0.01,'base','pedestal',wait= True)
            self.objectlist =['box0','box1', 'box2','box3']
            for i in xrange(len(self.trash_loc_x)):
                self.psi.attachBox(self.objectlist[i], 0.05 ,0.05,0.06,self.trash_loc_x[i], self.trash_loc_y[i],self.trash_loc_z[i],'base','pedestal', wait = True)
        self.psi.waitForSync()
#        return self.trash_loc_x, self.trash_loc_y, self.trash_loc_z

    def addTrashcan(self):
        self.psi.attachBox("table", 0.75, 1.52, 0.73, 0.84, 0.2, -0.55, 'base', 'pedestal', wait=True)
        self.psi.attachBox("trashcan",0.365, 0.265,0.39,1.03,-0.415, 0.01,'base','pedestal',wait= True)
        self.tc = PoseStamped()
        self.tc.header.frame_id = "camera_rgb_optical_frame"
        self.tc.header.stamp = rospy.Time.now()
        self.tc.pose.position.x = 1.03
        self.tc.pose.position.y = -0.415
        self.tc.pose.position.z = 0.1
        self.tc.pose.orientation.x = 0
        self.tc.pose.orientation.y = 1
        self.tc.pose.orientation.z = 0
        self.tc.pose.orientation.w =0
        return self.tc
        
    def moveTrashIntoTrashcan(self):
#        self.tc = SceneObstacles().addTrashcan()
#        self.littleboxes= SceneObstacles().addTrashAsObstacles()
        
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_start_state_to_current_state()
        self.group.set_planning_time(10)
        self.group.set_num_planning_attempts(10)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(1)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.01)
        self.pose_t = Pose()
        self.pose_t.orientation.x = 1
        self.pose_t.orientation.y = 0
        self.pose_t.orientation.z = 0
        self.pose_t.orientation.w = 0
        self.pose_t.position.x = 0.4
        self.pose_t.position.y = 0.1
        self.pose_t.position.z = 0.3
        self.group.set_pose_target(self.pose_t)
        self.plan = self.group.plan()
        rospy.sleep(5)
        self.group.go(wait =True)
        
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(self.plan)
        rospy.sleep(5)  
    


def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("TagsPose", anonymous=True)
#    ic = TagsPose()
#    ic = MoveBaxter()
    ic = SceneObstacles()
#    x = ic.getDictofPoses()
#    x = ic.transform_pose()
#    x = ic.makeDictofTransformedPoses()
#    x = ic.moveArm()
#    x = ic.addTrashAsObstacles()
    x = ic.moveTrashIntoTrashcan()
    print x
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)