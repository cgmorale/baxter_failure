#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from grasps_server import GraspClass
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_python import PlanningSceneInterface as MoveitPSI
from moveit_msgs.msg import PickupAction, PickupGoal, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
import copy
import moveit_commander
import moveit_msgs.msg
from math import pi
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule



moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
         code = MoveItErrorCodes.__dict__[name]
         moveit_error_dict[code] = name


def createPickupGoal(group = "right_arm", target = "box0", grasp_pose = PoseStamped(), possible_grasps = [], links_to_allow_contact = None):
    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 10.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 5.0
    pug.allowed_touch_objects = []
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)
    return pug
    

def createPlaceGoal(place_pose, place_locations, group = "right_arm", target ="box0", links_to_allow_contact= None):
    pg = PlaceGoal()
    pg.group_name = group
    pg.attached_object_name = target
    pg.place_locations = place_locations
    pg.allowed_planning_time = 10.0
    pg.planning_options.planning_scene_diff.is_diff = True
    pg.planning_options.planning_scene_diff.robot_state.is_diff= True
    pg.planning_options.plan_only = False
    pg.planning_options.replan = True
    pg.planning_options.replan_attempts = 5.0
    pg.allowed_touch_objects = ['<octomap>']
    pg.allowed_touch_objects.extend(links_to_allow_contact)
    return pg


class SceneObstacles():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.psi = PlanningSceneInterface()
        self.mpsi = MoveitPSI("base")
        self.mpsi.clear()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        #print self.group.get_current_joint_values()
        self.group.get_planning_frame()
        self.group.get_end_effector_link()
        self.mpsi.clear()
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        self.size = []
        self.orien_trash = []
        self.objlist = ['table','trashcan','box0', 'box1', 'box2','box3']
        for i in xrange(len( self.objlist)):
            self.mpsi.removeCollisionObject(self.objlist[i])
        

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
            self.mpsi.clear()
            self.mpsi.addBox('table',0.75, 1.52,0.73, 0.84, 0.2, -0.55, wait = True)
            self.mpsi.addBox('trashcan', 0.365, 0.265,0.39, 1.03, -0.415, 0.01, wait =True)
            self.objectlist =['box0','box1', 'box2','box3']
            for i in xrange(len(self.trash_loc_x)):
                self.mpsi.addBox(self.objectlist[i], 0.05, 0.05, 0.06, self.trash_loc_x[i], self.trash_loc_y[i], self.trash_loc_z[i], wait =True)
            return self.trash_loc_x, self.trash_loc_y, self.trash_loc_z

    def moveTrashIntoTrashcan(self):
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
        self.mpsi.attachBox('box0', 0.05, 0.05, 0.06, self.locx[0],self.locy[0],self.locz[0], self.frameattached,self.frameoktocollide, wait = True)
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
        self.mpsi.removeCollisionObject('box0', wait = True)


class PickAndPlaceServer():
    def __init__(self):
        rospy.loginfo("Initializing PickAndPlaceServer...")
        self.gc = GraspClass()
        rospy.loginfo("Connecting to pickup action server")
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Successfully connected")
        rospy.loginfo("Connecting to place action server")
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo("Successfully connected.")
        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")
        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected!")
                
        self.links_to_allow_contact = 'r_gripper_l_finger r_gripper_r_finger'
        self.pick_as= SimpleActionServer('/pickup_pose', PickUpPoseAction, execute_cb = self.pick_cb, auto_start = False)
        self.pick_as.start()
        
        self.place_as = SimpleActionServer('/place_pose', PickUpPoseAction, execute_cb = self.place_cb,  auto_start = False)
        self.place_as.start()
        self.psi = PlanningSceneInterface()
        self.mpsi = MoveitPSI("base")
        
    def pick_cb(self, goal):
        error_code = self.grasp_object(goal.object_pose)
        p_res = PickUpPoseResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)
            
    def place_cb(self, goal):
        error_code = self.place_object(goal.object_pose)
        p_res = PickUpPoseResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.place_as.set_aborted(p_res)
        else:
            self.place_as.set_succeeded(p_res)
    

    def wait_for_planning_scene_object(self, object_name = "box0"):
        rospy.loginfo("Waiting for object' " + object_name + " '' to appear in planning scene...")
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
        
        part_in_scene= False
        while not rospy.is_shutdown() and not part_in_scene:
            gps_resp = self.scene_srv.call(gps_req)
            for collision_obj in gps_resp.scene.world.collision_objects:
                if collision_obj.id == object_name:
                    part_in_scene = True
                    break 
                else:
                    rospy.sleep(1.0)
                    
        rospy.loginfo("'" + object_name + "'' is in scene!")
        
        
    def grasp_object(self, object_pose):
        self.trash_loc_x, self.trash_loc_y, self.trash_loc_z = SceneObstacles().addTrashAsObstacles()
        rospy.loginfo("Removing any previous object")
        self.scene.remove_attached_object("right_arm")
        self.scene.remove_world_object("box0")
        self.scene.remove_world_object("table")
        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv(EmptyRequest())
        rospy.sleep(2.0)
        rospy.loginfo("Adding new object")
        
        #### TODO: add scene obstacles from code in detectobjectusing tags
#        self.psi.clear()
        self.mpsi.addBox('table',0.75, 1.52,0.73, 0.84, 0.2, -0.55, wait = True)
        self.mpsi.addBox('trashcan', 0.365, 0.265,0.39, 1.03, -0.415, 0.01, wait =True)
        self.objectlist =['box0','box1', 'box2','box3']
        for i in xrange(len(self.objectlist)):
            self.mpsi.addBox(self.objectlist[i], 0.05, 0.05, 0.06, self.trash_loc_x[i], self.trash_loc_y[i], self.trash_loc_z[i], wait =True)
        
        possible_grasps = self.gc.create_grasps_from_object_pose(object_pose)
        goal = createPickupGoal("right_arm","box0", object_pose, possible_grasps, self.links_to_allow_contact)
        self.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.loginfo("Pick result:" + str(moveit_error_dict[result.error_code.val]))
        return result.error_code.val
        
        
    def place_object(self, object_pose):
        rospy.loginfo("Clearing octomap")
        self.clear_octomap_srv.call(EmptyRequest())
        possible_placings = self.gc.create_placings_from_object_pose(object_pose)
        rospy.loginfo("Trying to place using only arm")
        goal = createPlaceGoal(object_pose, possible_placings, "right_arm", "box0", self.links_to_allow_contact)
        self.place_ac.send_goal(goal)
        rospy.loginfo("Waiting for result")
        
        self.place_ac.wait_for_result()
        result = self.place_ac.get_result()
        rospy.loginfo(str(moveit_error_dict[result.error_code.val]))
        
        if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":
            rospy.loginfo("Trying to place with both arms")
            goal = createPlaceGoal(object_pose, possible_placings, "both_arms", "box0", self.links_to_allow_contact)
            rospy.loginfo("Sending goal")
            self.place_ac.send_goal(goal)
            rospy.loginfo("Waiting for result")
            self.place_ac.wait_for_result()
            result = self.place_ac.get_result()
            rospy.logerr(str(moveit_error_dict[result.error_code.val]))
            
        self.scene.remove_world_object("box0")
        return result.error_code.val
        
if __name__ =='__main__':
    rospy.init_node('service_pickandplace')
    paps = PickAndPlaceServer()
    rospy.spin()
    
            
        
        
        
        
        
        
        
        