#!/usr/bin/env python
import sys
#import os
import struct
import numpy as np
import rospy
import actionlib
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule
#from apriltags_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped,Pose, Vector3, Quaternion, Point
from std_msgs.msg import Header
import time
import copy
import tf
from moveit_python import PlanningSceneInterface
from math import pi, radians, sin, cos, sqrt
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, PlaceGoal, PlaceAction, PickupAction,MoveItErrorCodes, PickupGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from tf.transformations import quaternion_from_euler , euler_from_quaternion, quaternion_multiply
from actionlib_msgs.msg import GoalStatus
#from dynamic_reconfigure.server import Server
#from tiago_pick_demo.cfg import SphericalGraspConfig

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1]=='_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


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
        self.psi.clear()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        #print self.group.get_current_joint_values()
        self.group.get_planning_frame()
        self.group.get_end_effector_link()
        MoveBaxter.openGripper(MoveBaxter())
#        self.psi.clear()
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        self.size = []
        self.orien_trash = []
        self.objlist = ['table','trashcan','box0', 'box1', 'box2','box3']
        for i in xrange(len( self.objlist)):
            self.psi.removeCollisionObject(self.objlist[i])
        
    def addTable(self):
        # attachBox (self, "name", sizex, sizey,sizez, x, y, z, thing to attach to, thing it shouldnt collide, wait = True)
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
            self.psi.addBox('table',0.75, 1.52,0.73, 0.84, 0.2, -0.55, wait = True)
            self.psi.addBox('trashcan', 0.365, 0.265,0.39, 1.03, -0.415, 0.01, wait =True)
            self.objectlist =['box0','box1', 'box2','box3']
            for i in xrange(len(self.trash_loc_x)):
                self.psi.addBox(self.objectlist[i], 0.05, 0.05, 0.06, self.trash_loc_x[i], self.trash_loc_y[i], self.trash_loc_z[i], wait =True)
            return self.trash_loc_x, self.trash_loc_y, self.trash_loc_z

    def moveTrashIntoTrashcan(self):
#        self.tc = SceneObstacles().addTrashcan()
        self.locx, self.locy, self.locz = SceneObstacles().addTrashAsObstacles()
        self.frameattached= "base"
        self.frameoktocollide = ['right_gripper','r_gripper_l_finger', 'r_gripper_r_finger']
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_start_state_to_current_state()
        self.group.set_planning_time(10)
        self.group.set_num_planning_attempts(10)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(1)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.01)
        self.waypoints = []
        self.pose_t =  self.group.get_current_pose().pose
        self.waypoints.append(self.pose_t) 
        self.pose_t = Pose()
        self.pose_t.orientation.x = 1
        self.pose_t.orientation.y = 0
        self.pose_t.orientation.z = 0
        self.pose_t.orientation.w = 0
        self.pose_t.position.x = self.locx[0]
        self.pose_t.position.y = self.locy[0]
        self.pose_t.position.z = self.locz[0] + 0.05
        self.waypoints.append(copy.deepcopy(self.pose_t))
#        self.group.set_pose_target(self.pose_t)
        self.psi.attachBox('box0', 0.05, 0.05, 0.06, self.locx[0],self.locy[0],self.locz[0], self.frameattached,self.frameoktocollide, wait = True)
        self.pose_t.position.z -=0.03 
        self.waypoints.append(copy.deepcopy(self.pose_t))
#        self.plan = self.group.plan()
        self.plan, self.fraction = self.group.compute_cartesian_path(self.waypoints, 0.01, 0.0, avoid_collisions=True)
        #print self.fraction
        rospy.sleep(5)
        self.group.execute(self.plan)
#        self.group.go(wait =True)
        
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(self.plan)
        rospy.sleep(5)
        self.psi.removeCollisionObject('box0', wait = True)
        MoveBaxter.closeGripper(MoveBaxter())
        
        
class Pick_and_Place:
    
    def __init__(self):
        self._table_object_name = 'table'
        self._grasp_object_name = 'box0'
        self._target = 'trashcan'
        self._group = 'right_arm'
        
        self._grasp_object_width = 0.05
        self.robot = moveit_commander.RobotCommander()
        self.psi = PlanningSceneInterface("base")
        rospy.sleep(1.0)
        self.psi.clear()
        self._arm_group = moveit_commander.MoveGroupCommander('right_arm')
#        self._gripper_group = moveit_commander.MoveGroupCommander('right_gripper')
        rospy.sleep(5.0)
        self._arm_group.get_planning_frame()
        self._arm_group.get_end_effector_link()
        
        self.objlist = ['table','trashcan','box0', 'box1', 'box2','box3']
        for i in xrange(len( self.objlist)):
            self.psi.removeCollisionObject(self.objlist[i])
        
        self._approach_retreat_desired_dist = 0.05
        self._approach_retreat_min_dist = 0.01
        
        self.locx, self.locy, self.locz = SceneObstacles().addTrashAsObstacles()
        self.frameattached= "base"
        self.frameoktocollide = ['right_gripper','r_gripper_l_finger', 'r_gripper_r_finger']
        self._arm_group.set_planner_id("RRTConnectkConfigDefault")
        self._arm_group.set_start_state_to_current_state()
        self._arm_group.set_planning_time(10)
        self._arm_group.set_num_planning_attempts(10)
        self._arm_group.allow_replanning(True)
        self._arm_group.set_max_velocity_scaling_factor(1)
        self._arm_group.set_goal_position_tolerance(0.01)
        self._arm_group.set_goal_orientation_tolerance(0.01)
        
        self.dyn_rec_callback()
#        self.grasp_action()
        self.pickup_action()
        self.place_action()
    
    def normalize(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    def quaternion_from_vectors(self, v0, v1):
        if type(v0) == Point():
            v0 = [v0.x, v0.y, v0.z]
        if type(v1) == Point():
            v1 = [v1.x, v1.y, v1.z]

        v0 = self.normalize(v0)
        v1 = self.normalize(v1)
        c = np.cross(v0, v1)
        d = np.dot(v0, v1)
        try:
            s = sqrt((1.0 + d) * 2)
        except ValueError:
            s = 0.0
        if s == 0.0:
            # print "s == 0.0, we cant compute"
            return None  # [0.0, 0.0, 0.0, 1.0]

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = c[0] / s
        q[1] = c[1] / s
        q[2] = c[2] / s
        q[3] = s / 2.0
        return q
        
    def filter_poses(self, obj_poses, object_pose,filter_behind=False, filter_under=True):
        """Given the generated poses and the object pose
        filter out the poses that are behind or under (if set to True)
        :type sphere_poses: []
            list of Pose
            :type object_pose: PoseStamped
            :rtype: []"""
        new_list = []
        for pose in obj_poses:
        # if pose is further away than object, ditch it
            if filter_behind:
                if pose.position.x > object_pose.position.x:
                    continue
        # if pose if under the object, ditch it
            if filter_under:
                if pose.position.z < object_pose.position.z:
                    continue

            new_list.append(pose)
        return new_list

        
    def TrashPickUp(self):
        self.group = ['right_gripper']
        self.end_effector = ['r_gripper_l_finger','r_gripper_r_finger']
        pose = self.box0pose()
        grasps = self.create_grasps_from_object_pose(pose)
        self._pickup(self._target, self._grasp_object_width, grasps)
        self._place(self._group,self._grasp_object_name)
    
    def box0pose(self):
        self.locx, self.locy, self.locz = SceneObstacles().addTrashAsObstacles()
        
        self._pose_place = Pose()
        self._pose_place.position.x = self.locx[0] 
        self._pose_place.position.y = self.locy[0]
        self._pose_place.position.z = self.locz[0]
        self._pose_place.orientation.x = 1
        self._pose_place.orientation.y = 0
        self._pose_place.orientation.z = 0
        self._pose_place.orientation.w = 0
        return self._pose_place

    def dyn_rec_callback(self):
        self._grasp_postures_frame_id = "base"  
        self._gripper_joint_names = "r_gripper_l_finger r_gripper_r_finger"
        self._gripper_pre_grasp_positions = "0.038 0.038"
        self._time_pre_grasp_posture = 2.0
        self._time_grasp_posture = 1.0
        self._time_grasp_posture_final = 3.0 
        self._grasp_pose_frame_id = "base" 
        self._grasp_desired_distance = 0.50
        self._grasp_min_distance = 0.0 
        self._pre_grasp_direction_x = 1.0
        self._pre_grasp_direction_y = 0.0
        self._pre_grasp_direction_z = 0.0
        self._post_grasp_direction_x = -1.0
        self._post_grasp_direction_y = 0.0
        self._post_grasp_direction_z = 0.0
        self._grasp_quality = 0.1
        self._max_contact_force = 0.0 
        self._allowed_touch_objects =  "box0"
        self._fix_tool_frame_to_grasping_frame_roll = -90.0
        self._fix_tool_frame_to_grasping_frame_pitch = 0.0
        self._fix_tool_frame_to_grasping_frame_yaw = 0.0
        self._step_degrees_yaw = 15
        self._step_degrees_pitch=15
        self._min_degrees_yaw = 0
        self._max_degrees_yaw = 360
        self._min_degrees_pitch = 0
        self._max_degrees_pitch = 360
        return


    def create_grasps_from_object_pose(self, object_pose):
        """
        :type object_pose: PoseStamped
        """
        obj_poses = self.generate_grasp_poses(object_pose)
        filtered_poses = self.filter_poses(obj_poses, object_pose,filter_behind=False, filter_under=True)
        grasps = self.create_grasps_from_poses(filtered_poses)
        return grasps
    
    def create_grasps_from_poses(self, sphere_poses):
        """
        :type sphere_poses: []
            [] of Pose
        """
        grasps = []
        for idx, pose in enumerate(sphere_poses):
            grasps.append(
                self.create_grasp(pose,"grasp_" + str(idx)))
        return grasps
        
        
    def generate_grasp_poses(self, object_pose):
        # Compute all the points of the sphere with step X
        # http://math.stackexchange.com/questions/264686/how-to-find-the-3d-coordinates-on-a-celestial-spheres-surface
        self._grasp_desired_distance = 0.50
        self._step_degrees_yaw = 15
        self._step_degrees_pitch=15
        self._min_degrees_yaw = 0
        self._max_degrees_yaw = 360
        self._min_degrees_pitch = 0
        self._max_degrees_pitch = 360
        
        
        radius = self._grasp_desired_distance
        ori_x = 0.0
        ori_y = 0.0
        ori_z = 0.0
        obj_poses = []
        rotated_q = quaternion_from_euler(0.0, 0.0, radians(180))
        # altitude is yaw
        for altitude in range(self._min_degrees_yaw, self._max_degrees_yaw, self._step_degrees_yaw):
            altitude = radians(altitude)
            # azimuth is pitch
            for azimuth in range(self._min_degrees_pitch, self._max_degrees_pitch, self._step_degrees_pitch):
                azimuth = radians(azimuth)
                # This gets all the positions
                x = ori_x + radius * cos(azimuth) * cos(altitude)
                y = ori_y + radius * sin(altitude)
                z = ori_z + radius * sin(azimuth) * cos(altitude)
                # this gets all the vectors pointing outside of the center
                # quaternion as x y z w
                q = self.quaternion_from_vectors([radius, 0.0, 0.0], [x, y, z])
                # Cannot compute so the vectors are parallel
                if q is None:
                    # with this we add the missing arrow
                    q = rotated_q
                # We invert the orientations to look inwards by multiplying
                # with a quaternion 180deg rotation on yaw
                q = quaternion_multiply(q, rotated_q)

                # We actually want roll to be always 0.0 so we approach
                # the object with the gripper always horizontal
                # this rotation can be tuned with the dynamic params
                # multiplying later on
                roll, pitch, yaw = euler_from_quaternion(q)
                q = quaternion_from_euler(radians(0.0), pitch, yaw)

                x += object_pose.position.x
                y += object_pose.position.y
                z += object_pose.position.z
                current_pose = Pose(Point(x, y, z), Quaternion(*q))
                obj_poses.append(current_pose)
                return obj_poses

    def create_grasp(self, pose, grasp_id):
        """
        :type pose: Pose
            pose of the gripper for the grasp
        :type grasp_id: str
            name for the grasp
        :rtype: Grasp
        """
        self._grasp_postures_frame_id = "base"
        self._pre_grasp_direction_x = 1.0
        self._pre_grasp_direction_y = 0.0
        self._pre_grasp_direction_z = 0.0
        self._post_grasp_direction_x = -1.0
        self._post_grasp_direction_y = 0.0
        self._post_grasp_direction_z = 0.0
        self._gripper_joint_names = "r_gripper_l_finger r_gripper_r_finger"
        self._time_pre_grasp_posture = 2.0
        self._time_grasp_posture = 1.0
        self._time_grasp_posture_final = 3.0
        self._grasp_desired_distance = 0.50
        self._grasp_min_distance = 0.0
        self._max_contact_force = 0.0 
        self._allowed_touch_objects =  "box0"
        self._fix_tool_frame_to_grasping_frame_roll = -90.0
        self._fix_tool_frame_to_grasping_frame_pitch = 0.0
        self._fix_tool_frame_to_grasping_frame_yaw = 0.0
        self._gripper_grasp_positions= "0.015 0.015"

        g = Grasp()
        g.id = grasp_id

        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = self._grasp_postures_frame_id
        pre_grasp_posture.joint_names = [
            name for name in self._gripper_joint_names.split()]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [
            float(pos) for pos in self._gripper_pre_grasp_positions.split()]
        jtpoint.time_from_start = rospy.Duration(self._time_pre_grasp_posture)
        pre_grasp_posture.points.append(jtpoint)

        grasp_posture = copy.deepcopy(pre_grasp_posture)
        grasp_posture.points[0].time_from_start = rospy.Duration(
            self._time_pre_grasp_posture + self._time_grasp_posture)
        jtpoint2 = JointTrajectoryPoint()
        jtpoint2.positions = [
            float(pos) for pos in self._gripper_grasp_positions.split()]
        jtpoint2.time_from_start = rospy.Duration(
            self._time_pre_grasp_posture +
            self._time_grasp_posture + self._time_grasp_posture_final)
        grasp_posture.points.append(jtpoint2)

        g.pre_grasp_posture = pre_grasp_posture
        g.grasp_posture = grasp_posture

        header = Header()
        header.frame_id = self._grasp_pose_frame_id  # base_footprint
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        # Fix orientation from gripper_link to parent_link (tool_link)
        fix_tool_to_gripper_rotation_q = quaternion_from_euler(
            radians(self._fix_tool_frame_to_grasping_frame_roll),
            radians(self._fix_tool_frame_to_grasping_frame_pitch),
            radians(self._fix_tool_frame_to_grasping_frame_yaw)
        )
        q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
        fixed_pose = copy.deepcopy(pose)
        fixed_pose.orientation = Quaternion(*q)

        g.grasp_pose = PoseStamped(header, fixed_pose)
        g.grasp_quality = self._grasp_quality

        g.pre_grasp_approach = GripperTranslation()
        g.pre_grasp_approach.direction.vector.x = self._pre_grasp_direction_x
        g.pre_grasp_approach.direction.vector.y = self._pre_grasp_direction_y
        g.pre_grasp_approach.direction.vector.z = self._pre_grasp_direction_z
        g.pre_grasp_approach.direction.header.frame_id = self._grasp_postures_frame_id
        g.pre_grasp_approach.desired_distance = self._grasp_desired_distance
        g.pre_grasp_approach.min_distance = self._grasp_min_distance
        g.post_grasp_retreat = GripperTranslation()
        g.post_grasp_retreat.direction.vector.x = self._post_grasp_direction_x
        g.post_grasp_retreat.direction.vector.y = self._post_grasp_direction_y
        g.post_grasp_retreat.direction.vector.z = self._post_grasp_direction_z
        g.post_grasp_retreat.direction.header.frame_id = self._grasp_postures_frame_id
        g.post_grasp_retreat.desired_distance = self._grasp_desired_distance
        g.post_grasp_retreat.min_distance = self._grasp_min_distance

        g.max_contact_force = self._max_contact_force
        g.allowed_touch_objects = self._allowed_touch_objects
        return g
    
    def create_placings_from_object_pose(self, posestamped):
        """ Create a list of PlaceLocation of the object rotated every 15deg"""
        place_locs = []
        pre_grasp_posture = JointTrajectory()
        # Actually ignored....
        pre_grasp_posture.header.frame_id = self._grasp_pose_frame_id
        pre_grasp_posture.joint_names = [
            name for name in self._gripper_joint_names.split()]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [
            float(pos) for pos in self._gripper_pre_grasp_positions.split()]
        jtpoint.time_from_start = rospy.Duration(self._time_pre_grasp_posture)
        pre_grasp_posture.points.append(jtpoint)
        # Generate all the orientations every step_degrees_yaw deg
        for yaw_angle in np.arange(0.0, 2.0 * pi, radians(self._step_degrees_yaw)):
            pl = PlaceLocation()
            pl.place_pose = posestamped
            newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
            pl.place_pose.pose.orientation = Quaternion(
                newquat[0], newquat[1], newquat[2], newquat[3])
            # the frame is ignored, this will always be the frame of the gripper
            # so arm_tool_link
            pl.pre_place_approach = self.createGripperTranslation(
                Vector3(1.0, 0.0, 0.0))
            pl.post_place_retreat = self.createGripperTranslation(
                Vector3(-1.0, 0.0, 0.0))

            pl.post_place_posture = pre_grasp_posture
            place_locs.append(pl)

        return place_locs

    def createGripperTranslation(self, direction_vector, desired_distance=0.05, min_distance=0.01):
        # Gripper translation message with the direction vector, desired distance and minimum distance
        # to fill the pre_grasp_approach and post_grasp_retreat field in the Grasp message
        self.g_trans = GripperTranslation()
        self.g_trans.direction.header.frame_id = "base"
        self.g_trans.direction.header.stamp = rospy.Time.now()
        self.g_trans.direction.vector.x = direction_vector.x
        self.g_trans.direction.vector.y = direction_vector.y
        self.g_trans.direction.vector.z = direction_vector.z
        self.g_trans.desired_distance = desired_distance
        self.g_trans.min_distance = min_distance
        return self.g_trans 
    
#    def getPreGraspPosture(self):
#        self.pre_grasp_posture = JointTrajectory()
#        self.pre_grasp_posture.header.frame_id= "base"
#        self.pre_grasp_posture.header.stamp= rospy.Time.now()
#        self.pre_grasp_posture.joint_names = ['right_gripper','r_gripper_l_finger', 'r_gripper_r_finger']
#        self.pos = JointTrajectoryPoint()
#        self.pos.time_from_start = rospy.Duration(3.0)
#        #self.pos.positions - can add the specific positions for the joints
#        #self.pre_grasp_posture.points.append(self.pos)
#        return self.pre_grasp_posture
    
    def createPlaceLocation(self):
        self.place_locs = []
        for angle in np.arange(0,2*pi, radians(15)):
            
            self.pl = PlaceLocation()
        
            self.pl.place_pose = PoseStamped()
            self.pl.place_pose.pose.position.x = 1.08
            self.pl.place_pose.pose.position.y = -0.45
            self.pl.place_pose.pose.position.z = 0.06
            q = quaternion_from_euler(0.0,0.0,angle)
            self.pl.place_pose.pose.orientation = Quaternion(*q)
            self.pl.place_pose.header.frame_id = "base"
            self.pl.place_pose.header.stamp = rospy.Time.now()
        
            self.pl.pre_place_approach = self.createGripperTranslation(Vector3(0,0,-1.0))
            self.pl.post_place_retreat = self.createGripperTranslation(Vector3(0,0,1.0))
            self.place_locs.append(self.pl)
#        self.pl.pose_place_posture =self.getPreGraspPosture()
        return self.place_locs
        
    
    def _create_place_goal(self,group,target,places):
        self.psi.attachBox('box0', 0.05, 0.05, 0.06, self.locx[0],self.locy[0],self.locz[0], self.frameattached,self.frameoktocollide, wait = True)
        goal = PlaceGoal()
        goal.group_name = group
        goal.attached_object_name = target
        
        goal.place_locations.extend(places)
        
        goal.allowed_planning_time = 5.0
        
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.allow_gripper_support_collision = False
        goal.planning_options.replan_attempts = 10
        
        return goal
        
    
    def _create_pickup_goal(self,group,target, grasps):
        self._table_object_name = 'table'
        
        goal = PickupGoal()
        goal.group_name = group
        goal.target_name = target
        
        goal.possible_grasps.extend(grasps)
        
        goal.allowed_touch_objects.append(target)
        goal.support_surface_name = self._table_object_name
        goal.allowed_planning_time = 5.0
        
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10
        
        return goal 
        
    
#    def _generate_grasps(self,pose,width):
#        goal = GenerateGraspsGoal()
#        goal.pose =pose
#        goal.width = width
#        
#        options = GraspGeneratorOptions()
#        options.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
#        options.grasp_rotation = GraspGeneratorOptions.GRASP_ROTATION_FULL
#        
#        state = self._grasps_ac.send_goal_and_wait(goal)
#        
#        if state != GoalStatus.SUCCEEDED:
#            rospy.logerr('Grasp goal failed!: %s' % self._grasps_ac.get_goal_status_text())
#            return None
#            
#        grasps = self._grasps_ac.get_result().grasps
#        
#        return grasps
             
    def _pickup(self, target, width, grasps):
        MoveBaxter.openGripper(MoveBaxter())
        
        self._grasp_object_width = 0.05
        self._group = 'right_arm'
        
        goal = self._create_pickup_goal(self._group,'box0', grasps)
        print goal
        state = self._pickup_ac.send_goal_and_wait(goal)

        print state
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Pick up goal failed!: %s' %self._pickup_ac.get_goal_status_text())
            return None
            
        result = self._pickup_ac.get_result()
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot place target %s!: %s' % (self._group, 'box0', str(moveit_error_dict[err])))
            return False
        return True
    
    def _place(self,group,target):
        
        places = self.createPlaceLocation()
        goal = self._create_place_goal(group,target,places)
        state = self._place_ac.send_goal_and_wait(goal)
        
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Place goal failed!: %s' % self._place_ac.get_goal_status_text())
            return None
        
        result = self._place_ac.get_result()
        
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot place target %s!: %s' %(group, target, str(moveit_error_dict[err])))
            return False
        return True
            

    def pickup_action(self):
        self._grasp_object_width = 0.05
        self._grasp_object_name = 'box0'
        self._pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
        if not self._pickup_ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr('Pick up action client not available!')
            rospy.signal_shutdown('Pick up action client not available!')
            return
#        while not self._pickup( self._grasp_object_name, self._grasp_object_width):
#            rospy.logwarn('Pick up failed! Retrying...')
#            rospy.sleep(1.0)
#        rospy.loginfo('Pick up successfully!')
        
#    def grasp_action(self):
#        self._grasps_ac= actionlib.SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
#        if not self._grasps_ac.wait_for_server(rospy.Duration(5.0)):
#            rospy.logerr('Grasp generator action client not available!')
#            rospy.signal_shutdown('Grasp generator action client not available!')
#            return 
    
    def place_action(self):
        self._place_ac = actionlib.SimpleActionClient('/place',PlaceAction)
        if not self._place_ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr('Place action client not available!')
            rospy.signal_shutdown('Place action client not available!')
            return
#        while not self._place(self._arm_group,self._grasp_object_name,self._pose_place):
#            rospy.logwarn('Place failed! Retrying...')
#            rospy.sleep(1.0)
#        rospy.loginfo('Place successfully!')
        

def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("TagsPose", anonymous=True)
#    ic = TagsPose()
#    ic = MoveBaxter()
#    ic = SceneObstacles()
    ic = Pick_and_Place()
#    x = ic.getDictofPoses()
#    x = ic.transform_pose()
#    x = ic.makeDictofTransformedPoses()
#    x = ic.moveArm()
#    x = ic.addTrashAsObstacles()
#    x = ic.moveTrashIntoTrashcan()
#    x = ic.testMoveitGoal()
    x = ic.TrashPickUp()
    print x
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)