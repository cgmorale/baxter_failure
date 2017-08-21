#!/usr/bin/env python
import sys
import struct
#import numpy as np
import rospkg
import rospy
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped,Pose, Point
from std_msgs.msg import Header
import time
import copy
import tf
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from gazebo_msgs.srv import SpawnModel, DeleteModel


class TagsPose(object):
    def __init__(self):
        self.poses ={}
        self.limb = baxter_interface.Limb('right')
        self.tag_module = AprilTagModule()
        self.transformedDict = {}
        self.jointAngles = {}
        self.t = tf.TransformListener()
        frameId = 'camera_rgb_optical_frame'
        self.t.waitForTransform('base', frameId,rospy.Time(), rospy.Duration(2))
        time.sleep(1)
        ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        self.verbose = True
        
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
#        rospy.sleep(3.0)
        self.newPose= t.transformPose('base',ids)
        # change the orientation (quaternions) of april tags so that the IK can work
        # need to change it so Baxter knows were to grab the tags from
        self.newPose.pose.position.x -=0.00
        self.newPose.pose.position.y +=0.04
        self.newPose.pose.position.z -= 0.01
        self.newPose.pose.orientation.x = 0
        self.newPose.pose.orientation.y = 1.0
        self.newPose.pose.orientation.z = 0
        self.newPose.pose.orientation.w = 0
        return self.newPose


    def checkValidPose(self,poses):
        seq = 1
        time_stamp = rospy.Time.now()
        frame_id = 'base'
        hdr = Header(seq, time_stamp, frame_id)
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header= hdr, pose= poses))
        try:
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException),e:
            rospy.logger("Service call failed: %s" % (e,))
            return False
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0]!= resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER : 'User Provided Seed',
                        ikreq.SEED_CURRENT:'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                        }.get(resp_seeds[0],'None')
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self.verbose:
                print("Valid Joint Solution found from seed: {0}".format(seed_str))
                print("IK solution SUCCESS - Valid Joint Solution found: {0}".format(limb_joints))
        else:
            print("Invalid Pose - No valid joint solution found.")
            return False
        return limb_joints
        
    def makeDictofTransformedPoses(self):
        dictOfOldPoses = self.getDictofPoses()
        self.transformedDict = {}
        self.jointAngles = {}
        for key, val in dictOfOldPoses.items():
            transformed = self.transform_pose(val,self.t)
            validTransformed = self.checkValidPose(transformed.pose)
            if validTransformed != False:
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
        self.gripper.calibrate(block=True, timeout=5.0)
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
        

class PickPlace(object):
    def __init__(self, limb):
        self.limb = baxter_interface.Limb(limb)
        self.retreatdistance = 0.10
        self.gripper = baxter_interface.Gripper(limb)
        self.verbose = True
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        try:
            rospy.wait_for_service(ns, 5.0)
            print ("Getting robot state")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 0
        self.rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self.init_state = self.rs.state().enabled
        print ("Robot is being enabled")
        self.rs.enable()
        MoveBaxter.calibrateGripper(MoveBaxter())
        self.trash_loc_x = []
        self.trash_loc_y = []
        self.trash_loc_z = []
        
        
    def startPosition(self, limb):
        if limb == "right":
            self.startAngles = {'right_w0':-0.67,
                            'right_w1':1.03,
                            'right_w2':0.5,
                            'right_e0':1.19,
                            'right_e1':1.94,
                            'right_s0':0.08,
                            'right_s1':1.0}
        else:
            self.startAngles = {'left_w0':0.67,
                            'left_w1':1.03,
                            'left_w2':-0.5,
                            'left_e0':-1.19,
                            'left_e1':1.94,
                            'left_s0':-0.08,
                            'left_s1':-1.0}
        print ("Moving arm to starting position..")
        self.move_to_joint_position(self.startAngles)
        MoveBaxter.openGripper(MoveBaxter())
        rospy.sleep(1.0)
    
    def move_to_joint_position(self, joint_angles):
        if joint_angles:
            self.limb.move_to_joint_positions(joint_angles)
        else:
            print("No joint angles provided")
    
    def approach(self, pose):
        approach = copy.deepcopy(pose)
        approach.position.z = approach.position.z + self.retreatdistance
        approach.orientation.x = 0
        approach.orientation.y = 1.0
        approach.orientation.z = 0.0
        approach.orientation.w = 0.0
        joint_angles = TagsPose.checkValidPose(TagsPose(),approach)
        self.move_to_joint_position(joint_angles)
    
    def retract(self):
        current_pose = self.limb.endpoint_pose()
        ik_pose= Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self.retreatdistance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = TagsPose.checkValidPose(TagsPose(),ik_pose)
        self.move_to_joint_position(joint_angles)
    
    
    def retract_from_trash(self):
        current_pose = self.limb.endpoint_pose()
        after_pose = Pose()
        after_pose.position.x = current_pose['position'].x
        after_pose.position.y = current_pose['position'].y
        after_pose.position.z = current_pose['position'].z + 0.04
        after_pose.orientation.x = current_pose['orientation'].x
        after_pose.orientation.y = current_pose['orientation'].y
        after_pose.orientation.z = current_pose['orientation'].z
        after_pose.orientation.w = current_pose['orientation'].w
        joint_angles = TagsPose.checkValidPose(TagsPose(),after_pose)
        self.move_to_joint_position(joint_angles)
        
        
    def servo_to_pose(self, pose):
        tag = copy.deepcopy(pose)
        tag.orientation.x = 0
        tag.orientation.y = 1.0
        tag.orientation.z = 0.0
        tag.orientation.w = 0.0
        joint_angles = TagsPose.checkValidPose(TagsPose(),tag)
        self.move_to_joint_position(joint_angles)
    
    def servo_to_trash(self,pose):
        joint_angles = TagsPose.checkValidPose(TagsPose(), pose)
        self.move_to_joint_position(joint_angles)
        
    def pick(self, pose):
        MoveBaxter.openGripper(MoveBaxter())
        self.approach(pose)
        self.servo_to_pose(pose)
        MoveBaxter.closeGripper(MoveBaxter())
        self.retract()
        
    def place(self,pose):
        self.approach(pose)
        self.servo_to_trash(pose)
        MoveBaxter.openGripper(MoveBaxter())
        self.retract_from_trash()
        
    def load_gazebo_models(self, table_pose=Pose(position=Point(x=0.84, y=0.2, z=-0.55)),
                       table_reference_frame="base"):
        
#        self.trashposes,self.baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
#        for key, val in self.trashposes.items():
#            val = self.trashposes[klimbey]
#            self.trashloc = val.pose
#            self.trash_loc_x.append(self.trashloc.position.x)
#            self.trash_loc_y.append(self.trashloc.position.y)
#            self.trash_loc_z.append(self.trashloc.position.z)
            
        block_pose = Pose()
        block_pose.position.x = 0.3 #self.trash_loc_x[0]
        block_pose.position.y = 0.1 #self.trash_loc_y[0]
        block_pose.position.z = -0.16 #self.trash_loc_z[0]
        block_pose.orientation.x = 0.0
        block_pose.orientation.y = 0.0
        block_pose.orientation.z = 0.0
        block_pose.orientation.w = 1.0
        block_reference_frame = "base"
    # Get Models' Path
        model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
        table_xml = ''
        with open (model_path + "cafe_table/model.sdf", "r") as table_file:
            table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
        block_xml = ''
        with open (model_path + "block/model.urdf", "r") as block_file:
            block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))  
    
    def delete_gazebo_models(self):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("cafe_table")
            resp_delete = delete_model("block")
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
        
def main(args):
    rospy.init_node("pickandplaceikservice", anonymous=True)
    limb = "right"
#    PickPlace.load_gazebo_models(PickPlace(limb))
#    rospy.on_shutdown(PickPlace.delete_gazebo_models(PickPlace(limb)))
    pnp = PickPlace(limb)
    trash_loc_x = []
    trash_loc_y = []
    trash_loc_z = []
    trashposes, baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
    for key, val in trashposes.items():
        val = trashposes[key]
        trashloc = val.pose
        trash_loc_x.append(trashloc.position.x)
        trash_loc_y.append(trashloc.position.y)
        trash_loc_z.append(trashloc.position.z)
    for i in xrange(len(trash_loc_x)): 
        block_pose = Pose()
        block_pose.position.x = trash_loc_x[i]
        block_pose.position.y = trash_loc_y[i]
        block_pose.position.z = trash_loc_z[i]
        block_pose.orientation.x = 0.0
        block_pose.orientation.y = 0.0
        block_pose.orientation.z = 0.0
        block_pose.orientation.w = 1.0
        block_reference_frame = "base"
        pnp.pick(block_pose)
        trash_pose = Pose()
        trash_pose.position.x = 0.85
        trash_pose.position.y = -0.44
        trash_pose.position.z = 0.38
        trash_pose.orientation.x = 0.0968
        trash_pose.orientation.y = 0.96
        trash_pose.orientation.z = -0.06
        trash_pose.orientation.w = 0.27
        pnp.place(trash_pose)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)