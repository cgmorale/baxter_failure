# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
import numpy as np
import math
from math import radians, pi
import copy
from copy import deepcopy

import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from visualization_msgs.msg import MarkerArray, Marker

from tf import transformations
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_about_axis, unit_vector, quaternion_multiply


def normalize(v):
    norm = np.linalg.norm(v)
    if norm ==0:
        return v
    return v/norm
    

def quaternion_from_vectors(v1,v2):
    if type(v1)==Point():
        v1 = [v1.x, v1.y, v1.z]
    if type(v2)==Point():
        v2 = [v2.x, v2.y, v2.z]
        
    v1 = normalize(v1)
    v2 = normalize(v2)
    
    crossp = np.cross(v1,v2)
    dotp = np.dot(v1,v2)
    
    try: 
        s = math.sqrt((1.0 + dotp)*2)
    except ValueError:
        s = 0.0
    if s ==0.0:
        return None
    
    quat = [0.0, 0.0, 0.0, 0.0]
    quat[0] = crossp[0]/s
    quat[1] = crossp[1]/s
    quat[2] = crossp[2]/s
    quat[3] = s /2.0
    
    return quat
    
def filter_poses(poses, object_pose, filter_behind = False, filter_under = True):
    new_list = []
    for pose in poses:
        if filter_behind:
            if pose.position.x> object_pose.pose.position.x:
                continue
        if filter_under:
            if pose.position.z < object_pose.pose.position.z:
                continue
        new_list.append(pose)
    return new_list
    
class GraspClass():
    
    def __init__(self):
        rospy.loginfo("Initializing Grasps...")
        self.grasp_desired_distance = 0.15
        
        self.max_degrees_yaw = 360
        self.min_degrees_yaw = 0
        self.step_degrees_yaw = 10
        
        self.max_degrees_pitch = 360
        self.min_degrees_pitch = 0
        self.step_degrees_pitch = 10
        
        self.gripper_joint_names = 'r_gripper_l_finger r_gripper_r_finger'
        self.gripper_pre_grasp_positions = "0.038 0.038"
        
        self.time_pre_grasp_posture = 2.0
        self.time_grasp_posture = 1.0
        self.time_grasp_posture_final = 3.0
        
        self.gripper_grasp_positions = "0.015 0.015"
        
        self.grasp_quality = 0.1
        
        self.fix_tool_frame_to_grasping_frame_roll = -90.0
        self.fix_tool_frame_to_grasping_frame_pitch= 0.0
        self.fix_tool_grame_to_grasping_frame_yaw = 0.0
        
        self.grasp_pose_frame_id = "base"
        self.grasp_postures_frame_id = "base" # check the frame 
        
        self.pre_grasp_direction_x = 1.0
        self.pre_grasp_direction_y = 0.0
        self.pre_grasp_direction_z = 0.0
        
        self.grasp_desired_distance = 0.20
        self.grasp_min_distance= 0.0
        
        self.post_grasp_direction_x = -1.0
        self.post_grasp_direction_y = 0.0
        self.post_grasp_direction_z = 0.0
        
        self.max_contact_force = 1.0
        self.allowed_touch_objects = "box0"
        
    
    def generate_grasp_poses(self, object_pose):
        radius = self.grasp_desired_distance
        orix = 0.0
        oriy = 0.0
        oriz = 0.0
        
        poses = []
        rotated_quat = quaternion_from_euler(0.0, 0.0, math.radians(180))
        
        yaw = int((self.max_degrees_yaw - self.min_degrees_yaw)/ self.step_degrees_yaw)
        pitch = int((self.max_degrees_pitch - self.min_degrees_pitch)/self.step_degrees_pitch)
        
        # altitude is yaw
        for altitude in range(self.min_degrees_yaw , self.max_degrees_yaw, self.step_degrees_yaw):
            altitude = math.radians(altitude)
            
            for azimuth in range(self.min_degrees_pitch, self.max_degrees_pitch, self.step_degrees_pitch):
                azimuth = math.radians(azimuth)
                
                x = orix + radius * math.cos(azimuth)*math.cos(altitude)
                y = oriy + radius * math.sin(altitude)
                z = oriz + radius * math.sin(azimuth)*math.cos(altitude)
                
                q = quaternion_from_vectors([radius, 0.0, 0.0], [x,y,z])
                if q is None:
                    q = rotated_quat
                    
                q = quaternion_multiply(q, rotated_quat)
                
                roll, pitch, yaw = euler_from_quaternion(q)
                q = quaternion_from_euler(math.radians(0.0),pitch, yaw)
                
                x += object_pose.pose.position.x
                y += object_pose.pose.position.y
                z += object_pose.pose.position.z
                current_pose = Pose(Point(x,y,z), Quaternion(*q))
                poses.append(current_pose)
            return poses
                    
                    
    def create_grasps_from_poses(self,poses):
        grasps = []
        for idx, pose in enumerate(poses):
            grasps.append(self.create_grasps(pose, "grasp_" + str(idx)))
        return grasps
        
    def create_grasps(self, pose, grasp_id):
        g = Grasp()
        g.id = grasp_id
            
        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = self.grasp_postures_frame_id
        pre_grasp_posture.joint_names = [name for name in self.gripper_joint_names.split()]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [float(pos) for pos in self.gripper_pre_grasp_positions.split()]
        jtpoint.time_from_start = rospy.Duration(self.time_pre_grasp_posture)
        pre_grasp_posture.points.append(jtpoint)
        
        grasp_posture = copy.deepcopy(pre_grasp_posture)
        grasp_posture.points[0].time_from_start= rospy.Duration(self.time_pre_grasp_posture + self.time_grasp_posture)
        jtpoint2 = JointTrajectoryPoint()
        jtpoint2.positions = [float(pos) for pos in self.gripper_grasp_positions.split()]
        jtpoint2.time_from_start = rospy.Duration(self.time_pre_grasp_posture + self.time_grasp_posture + self.time_grasp_posture_final)
        grasp_posture.points.append(jtpoint2)
            
        g.pre_grasp_posture = pre_grasp_posture
        g.grasp_posture = grasp_posture
            
        header = Header()
        header.frame_id = self.grasp_pose_frame_id
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        fix_tool_to_gripper_rotation_q = quaternion_from_euler(
            math.radians(self.fix_tool_frame_to_grasping_frame_roll),
            math.radians(self.fix_tool_frame_to_grasping_frame_pitch),
            math.radians(self.fix_tool_grame_to_grasping_frame_yaw))
        q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
        fixed_pose = copy.deepcopy(pose)
        fixed_pose.orientation = Quaternion(*q)
            
        g.grasp_pose = PoseStamped(header,fixed_pose)
        g.grasp_quality = self.grasp_quality
            
        g.pre_grasp_approach = GripperTranslation()
        g.pre_grasp_approach.direction.vector.x = self.pre_grasp_direction_x
        g.pre_grasp_approach.direction.vector.y = self.pre_grasp_direction_y
        g.pre_grasp_approach.direction.vector.z = self.pre_grasp_direction_z
        g.pre_grasp_approach.direction.header.frame_id = self.grasp_postures_frame_id 
        g.pre_grasp_approach.desired_distance = self.grasp_desired_distance
        g.pre_grasp_approach.min_distance = self.grasp_min_distance
        g.post_grasp_retreat = GripperTranslation()
        g.post_grasp_retreat.direction.vector.x = self.post_grasp_direction_x
        g.post_grasp_retreat.direction.vector.y= self.post_grasp_direction_y
        g.post_grasp_retreat.direction.vector.z = self.post_grasp_direction_z
        g.post_grasp_retreat.direction.header.frame_id = self.grasp_postures_frame_id
        g.post_grasp_retreat.desired_distance = self.grasp_desired_distance
        g.post_grasp_retreat.min_distance = self.grasp_min_distance
            
        g.max_contact_force= self.max_contact_force
        g.allowed_touch_objects = self.allowed_touch_objects
        return g
            
    def create_grasps_from_object_pose(self, object_pose):
        poses = self.generate_grasp_poses(object_pose)
        filtered_poses = filter_poses(poses, object_pose, filter_behind = False, filter_under = True)
        grasps = self.create_grasps_from_poses(filtered_poses)
        return grasps
            
    def create_placings_from_object_pose(self, posestamped):
        place_locs = []
        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = self.grasp_pose_frame_id
        pre_grasp_posture.joint_names = [name for name in self.gripper_joint_names.split()]
        jtpoint = JointTrajectoryPoint()
        jtpoint.positions = [float(pos) for pos in self.gripper_pre_grasp_positions.split()]
        jtpoint.time_from_start = rospy.Duration(self.time_pre_grasp_posture)
        pre_grasp_posture.points.append(jtpoint)
            
        for yaw_angle in np.arange(0.0, 2.0*pi, radians(self.step_degrees_yaw)):
            pl = PlaceLocation()
            pl.place_pose = posestamped
            newquat = quaternion_from_euler(0.0,0.0, yaw_angle)
            pl.place_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
            pl.pre_place_approach = self.createGripperTranslation(Vector3(1.0, 0.0, 0.0))
            pl.post_place_retreat = self.createGripperTranslation(Vector3(-1.0, 0.0,0.0))
            pl.post_place_posture = pre_grasp_posture
            place_locs.append(pl)
        return place_locs


    def createGripperTranslation(self, direction_vector, desired_distance = 0.15, min_distance = 0.01):
        g_trans = GripperTranslation()
        g_trans.direction.header.frame_id = self.grasp_postures_frame_id
        g_trans.direction.header.stamp = rospy.Time.now()
        g_trans.direction.vector.x = direction_vector.x
        g_trans.direction.vector.y = direction_vector.y
        g_trans.direction.vector.z = direction_vector.z
        g_trans.desired_distance = desired_distance
        g_trans.min_distance = min_distance
        return g_trans

if __name__ =='__main__':
    rospy.init_node("grasps_server")
    g = GraspClass()
    ps = PoseStamped()
    ps.header.frame_id = 'base'
    ps.pose.position.x = 0.2
    ps.pose.position.y = 0.0
    ps.pose.position.z = 0.2
    ps.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
        g.create_grasps_from_object_pose(ps)
        rospy.sleep(1.0)
        
        
                
                
                
            
            