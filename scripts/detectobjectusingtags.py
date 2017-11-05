#!/usr/bin/env python
import sys
import struct
import rospkg
import rospy
import baxter_interface
from lab_ros_perception.AprilTagModule import AprilTagModule
from sensor_msgs.msg import Range, Image
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
import cv2
import cv_bridge
import std_srvs.srv
import threading


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
        print self.newPose.pose
        self.newPose.pose.position.x -=0.05
        self.newPose.pose.position.y +=0.04
#        self.newPose.pose.position.z -= 0.01
        self.newPose.pose.position.z = -0.155
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

class IRSensor(object):
    def __init__(self):
        self.distance= {}
        root_name = "/robot/range/"
        sensor_name = "right_hand_range/state"
        self._right_sensor = rospy.Subscriber(root_name + sensor_name, Range, callback = self._sensorCallback,callback_args = 'right', queue_size = 1)
        
    def _sensorCallback(self,msg, side):
        
        self.distance[side] = msg.range
        rangeofIR = self.distance.get('right')

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
        return
        
    def changeBaxterFace(self, path):
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)
    
  
    def open_camera(self, camera):
        if camera== "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera== "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("Error - invalid camera")
        cam.open()
        rospy.sleep(1.0)
        
    def close_camera(self, camera):
        if camera== "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera== "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("Error - invalid camera")
        cam.close()
        rospy.sleep(1.0)
        
    def camera_callback(self, data, camera_name):
        try:
            self.cv_image= cv_bridge.CvBridge().imgmsg_to_cv2(data,"bgr8")
        except cv_bridge.CvBridgeError,e:
            print e
    
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")
    
    def subscribe_to_camera(self,camera):
        if camera =="right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        else:
            sys.exit("Error- subscribe to camera is invalid")
        camera_sub =rospy.Subscriber(camera_str, Image, camera_callback)
        
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset',std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset',timeout = 10)
        reset_srv()
        

class PickPlace(object):
    def __init__(self):
        self.limb = baxter_interface.Limb("right")
        self.retreatdistance = 0.10
        self.gripper = baxter_interface.Gripper("right")
        self._ir_sensor = IRSensor()
        while "right" not in self._ir_sensor.distance:
            rospy.sleep(1)            
        self.verbose = True
        ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
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
        
#        MoveBaxter.reset_cameras(MoveBaxter())
##        MoveBaxter.close_camera(MoveBaxter(),"head")
#        MoveBaxter.open_camera(MoveBaxter(),"right")
#        MoveBaxter.subscribe_to_camera(MoveBaxter(),"right")
        
        
    def startPosition(self):
        self.startAngles = {'right_w0':-0.67,
                            'right_w1':1.03,
                            'right_w2':0.5,
                            'right_e0':1.19,
                            'right_e1':1.94,
                            'right_s0':0.08,
                            'right_s1':-1.0}
        print ("Moving arm to starting position..")
        self.move_to_joint_position(self.startAngles)
        MoveBaxter.openGripper(MoveBaxter())
        rospy.sleep(1.0)
    
    def move_to_joint_position(self, joint_angles):
        if joint_angles:
            self.limb.set_joint_position_speed(0.6)
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
        
    def approachBag(self,pose):
        aptrash= copy.deepcopy(pose)
        aptrash.position.z = aptrash.position.z + self.retreatdistance
        joint_angles = TagsPose.checkValidPose(TagsPose(), aptrash)
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
    
    
    
    def retract_from_bag(self):
        current_pose = self.limb.endpoint_pose()
        after_pose = Pose()
        after_pose.position.x = current_pose['position'].x
        after_pose.position.y = current_pose['position'].y
        after_pose.position.z = current_pose['position'].z + 0.20
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
    
    def servo_to_bag(self,pose):
        descend = copy.deepcopy(pose)
        descend.position.z -= 0.2
        joint_angles = TagsPose.checkValidPose(TagsPose(), descend)
        self.move_to_joint_position(joint_angles)
        
    def pick(self, pose):
        pickObject = True
        MoveBaxter.openGripper(MoveBaxter())
        self.approach(pose)
        self.servo_to_pose(pose)
        if self._ir_sensor.distance.get("right") < 0.17 and self._ir_sensor.distance.get("right")>0.13 :
            pose = copy.deepcopy(pose)
            pose.position.z -= 0.015
            self.servo_to_pose(pose)
        MoveBaxter.closeGripper(MoveBaxter())
        rospy.sleep(2.0)
        if self.gripper.position()<5.0:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/surprised.jpg')
            self.retract()
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/sad.jpg')
            pickObject= False
        self.retract()
        return pickObject
    
    def picktothrow(self,pose):
        MoveBaxter.openGripper(MoveBaxter())
        self.approach(pose)
        self.servo_to_pose(pose)
        if self._ir_sensor.distance.get("right") < 0.17 and self._ir_sensor.distance.get("right")>0.13 :
            pose = copy.deepcopy(pose)
            pose.position.z -= 0.015
            self.servo_to_pose(pose)
        MoveBaxter.closeGripper(MoveBaxter())
        rospy.sleep(2.0)
        if self.gripper.position()>5.0:
            self.retract()
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/angryred.jpg')
            self.openGripperToThrow()
            
    def picktoMisplace(self, pose):
        MoveBaxter.openGripper(MoveBaxter())
        self.approach(pose)
        self.servo_to_pose(pose)
        if self._ir_sensor.distance.get("right") < 0.17 and self._ir_sensor.distance.get("right")>0.13 :
            pose = copy.deepcopy(pose)
            pose.position.z -= 0.015
            self.servo_to_pose(pose)
        MoveBaxter.closeGripper(MoveBaxter())
        rospy.sleep(2.0)
        if self.gripper.position()<5.0:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/surprised.jpg')
            self.retract()
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/sad.jpg')
        self.retract()
        item_pose = Pose()
        item_pose.position.x = 1.100
        item_pose.position.y = -0.4
        item_pose.position.z = 0.25
        item_pose.orientation.x = 0.098
        item_pose.orientation.y = 0.96
        item_pose.orientation.z = -0.065
        item_pose.orientation.w = 0.27
        self.approachBag(item_pose)
        limb=baxter_interface.Limb("right")
        limb.set_joint_position_speed(0.5)
        endAngles = {'right_w0':-1.117,
                       'right_w1':-0.224,
                       'right_w2':1.469,
                       'right_e0':-0.588,
                       'right_e1':0.698,
                       'right_s0':0.324,
                       'right_s1':-0.574}
        limb.move_to_joint_positions(endAngles)
        MoveBaxter.openGripper(MoveBaxter())
        
    def movetoend(self):
        limb=baxter_interface.Limb("right")
        limb.set_joint_position_speed(1.0)
        endAngles = {'right_w0':-0.85,
                       'right_w1':0.26,
                       'right_w2':0.76,
                       'right_e0':0.436,
                       'right_e1':0.388,
                       'right_s0':0.708,
                       'right_s1':-0.622}
        limb.move_to_joint_positions(endAngles)
    
    def openg(self):
        rospy.sleep(1.5)
        self.gripper.command_position(position=100.0, block =False)
        self.gripper.open()
        
    def openGripperToThrow(self):
        threading.Thread(target= self.movetoend).start()
        threading.Thread(target= self.openg).start()
    
    def crazyMoves(self, pose):
        MoveBaxter.openGripper(MoveBaxter())
        self.approach(pose)
        self.servo_to_pose(pose)
        if self._ir_sensor.distance.get("right") < 0.17 and self._ir_sensor.distance.get("right")>0.13 :
            pose = copy.deepcopy(pose)
            pose.position.z -= 0.015
            self.servo_to_pose(pose)
        MoveBaxter.closeGripper(MoveBaxter())
        rospy.sleep(2.0)
        if self.gripper.position()>5.0:
            self.retract()
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/angryred.jpg')
        self.approachBag(pose)
        rospy.sleep(0.5)
        limb=baxter_interface.Limb("right")
        limb.set_joint_position_speed(1.0)
        firstAngles = {'right_w0':-0.78,
                       'right_w1':0.31,
                       'right_w2':0.74,
                       'right_e0':0.43,
                       'right_e1':0.79,
                       'right_s0':0.36,
                       'right_s1':-0.75}
        limb.move_to_joint_positions(firstAngles)
        secondAngles = {'right_w0':-0.76,
                       'right_w1':-0.02,
                       'right_w2':0.93,
                       'right_e0':-0.27,
                       'right_e1':0.80,
                       'right_s0':0.64,
                       'right_s1':-0.65}
        limb.move_to_joint_positions(secondAngles)
        rospy.sleep(0.5)
        thirdAngles = {'right_w0':-0.76,
                       'right_w1':-0.11,
                       'right_w2':1.10,
                       'right_e0':-0.54,
                       'right_e1':0.57,
                       'right_s0':1.45,
                       'right_s1':-0.42}
        limb.move_to_joint_positions(thirdAngles)
        rospy.sleep(0.5)
        fourthAngles = {'right_w0':-0.67,
                       'right_w1':1.03,
                       'right_w2':0.50,
                       'right_e0':1.18,
                       'right_e1':1.93,
                       'right_s0':0.08,
                       'right_s1':-0.99}
        limb.move_to_joint_positions(fourthAngles)
        rospy.sleep(0.5)
        MoveBaxter.openGripper(MoveBaxter())
        
    def place(self,pose):
        self.approachBag(pose)
        self.servo_to_bag(pose)
        MoveBaxter.openGripper(MoveBaxter())
        self.retract_from_bag()
        

        
def main(args):
    rospy.init_node("pickandplaceikservice", anonymous=True)
    rightl = baxter_interface.Limb('right')
    rightl.set_joint_position_speed(0.6)
    MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
    brs = IRSensor()
    
    number = input("choose state:")
    if number ==1:
        x = 1
        count = 0
        while x ==1:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
            pnp = PickPlace()
            item_loc_x = []
            item_loc_y = []
            item_loc_z = []
            itemposes, baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
            for key, val in itemposes.items():
                val = itemposes[key]
                itemloc = val.pose
                item_loc_x.append(itemloc.position.x)
                item_loc_y.append(itemloc.position.y)
                item_loc_z.append(itemloc.position.z)
            for i in xrange(len(item_loc_x)): 
                block_pose = Pose()
                block_pose.position.x = item_loc_x[i]
                block_pose.position.y = item_loc_y[i]
                block_pose.position.z = item_loc_z[i]
                block_pose.orientation.x = 0.0
                block_pose.orientation.y = 0.0
                block_pose.orientation.z = 0.0
                block_pose.orientation.w = 1.0               
                if (pnp.pick(block_pose)) == True:
                    item_pose = Pose()
                    if count > 5:
                        item_pose.position.x = 1.100
                        item_pose.position.y = -0.4
                        item_pose.position.z = 0.25
                        item_pose.orientation.x = 0.098
                        item_pose.orientation.y = 0.96
                        item_pose.orientation.z = -0.065
                        item_pose.orientation.w = 0.27
                    else:
                        item_pose.position.x = 0.85
                        item_pose.position.y = -0.44
                        item_pose.position.z = 0.28
                        item_pose.orientation.x = 0.0968
                        item_pose.orientation.y = 0.96
                        item_pose.orientation.z = -0.06
                        item_pose.orientation.w = 0.27
                    pnp.place(item_pose)
                else:
                    PickPlace.startPosition(PickPlace())
                count+= 1
    
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
    
    elif number ==2:
        x = 1
        count = 0
        while x ==1:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
            pnp = PickPlace()
            item_loc_x = []
            item_loc_y = []
            item_loc_z = []
            itemposes, baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
            for key, val in itemposes.items():
                if key == 16:
                    val16 = itemposes[key]
                    throwobj = val16.pose
                    throw_object = Pose()
                    throw_object.position.x = throwobj.position.x
                    throw_object.position.y = throwobj.position.y
                    throw_object.position.z = throwobj.position.z
                    throw_object.orientation.x = 0.0
                    throw_object.orientation.y = 0.0
                    throw_object.orientation.z = 0.0
                    throw_object.orientation.w = 1.0
                    pnp.picktothrow(throw_object)
                    rospy.sleep(5.0)
                    PickPlace.startPosition(PickPlace())
                elif key ==10:
                    val10 = itemposes[key]
                    misplaceobj = val10.pose
                    misplace_object = Pose()
                    misplace_object.position.x = misplaceobj.position.x
                    misplace_object.position.y = misplaceobj.position.y
                    misplace_object.position.z = misplaceobj.position.z
                    misplace_object.orientation.x = 0.0
                    misplace_object.orientation.y = 0.0
                    misplace_object.orientation.z = 0.0
                    misplace_object.orientation.w = 1.0
                    pnp.picktoMisplace(misplace_object)
                    rospy.sleep(5.0)
                    PickPlace.startPosition(PickPlace())
                elif key ==6:
                    val6 = itemposes[key]
                    missobj = val6.pose
                    miss_object = Pose()
                    miss_object.position.x = missobj.position.x
                    miss_object.position.y = missobj.position.y
                    miss_object.position.z = missobj.position.z + 0.1
                    miss_object.orientation.x = 0.0
                    miss_object.orientation.y = 0.0
                    miss_object.orientation.z = 0.0
                    miss_object.orientation.w = 1.0
                    pnp.pick(miss_object)
                    rospy.sleep(1.0)
                    othermiss = Pose()
                    othermiss.position.x = missobj.position.x + 0.15
                    othermiss.position.y = missobj.position.y
                    othermiss.position.z = missobj.position.z
                    othermiss.orientation.x = 0.0
                    othermiss.orientation.y = 0.0
                    othermiss.orientation.z = 0.0
                    othermiss.orientation.w = 1.0
                    pnp.pick(othermiss)
                    thirdmiss = Pose()
                    thirdmiss.position.x = missobj.position.x
                    thirdmiss.position.y = missobj.position.y + 0.15
                    thirdmiss.position.z = missobj.position.z
                    thirdmiss.orientation.x = 0.0
                    thirdmiss.orientation.y = 0.0
                    thirdmiss.orientation.z = 0.0
                    thirdmiss.orientation.w = 1.0
                    pnp.pick(thirdmiss)
                    fourth = Pose()
                    fourth.position.x = missobj.position.x
                    fourth.position.y = missobj.position.y
                    fourth.position.z = missobj.position.z
                    fourth.orientation.x = 0.0
                    fourth.orientation.y = 0.0
                    fourth.orientation.z = 0.0
                    fourth.orientation.w = 1.0
                    pnp.pick(fourth)
                    MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
                    itempos = Pose()
                    itempos.position.x = 0.85
                    itempos.position.y = -0.44
                    itempos.position.z = 0.28
                    itempos.orientation.x = 0.0968
                    itempos.orientation.y = 0.96
                    itempos.orientation.z = -0.06
                    itempos.orientation.w = 0.27
                    pnp.place(itempos)
                    rospy.sleep(5.0)
                    PickPlace.startPosition(PickPlace())
                elif key == 9:
                    val17 = itemposes[key]
                    movesobj = val17.pose
                    moves_object = Pose()
                    moves_object.position.x = movesobj.position.x
                    moves_object.position.y = movesobj.position.y
                    moves_object.position.z = movesobj.position.z
                    moves_object.orientation.x = 0.0
                    moves_object.orientation.y = 0.0
                    moves_object.orientation.z = 0.0
                    moves_object.orientation.w = 1.0
                    pnp.crazyMoves(moves_object)
                    rospy.sleep(5.0)
                    PickPlace.startPosition(PickPlace())
                elif key == 14:
                    val14 = itemposes[key]
                    throwobj = val14.pose
                    throw_object = Pose()
                    throw_object.position.x = throwobj.position.x
                    throw_object.position.y = throwobj.position.y
                    throw_object.position.z = throwobj.position.z
                    throw_object.orientation.x = 0.0
                    throw_object.orientation.y = 0.0
                    throw_object.orientation.z = 0.0
                    throw_object.orientation.w = 1.0
                    pnp.pick(throw_object)
                    item_pose = Pose()
                    item_pose.position.x =0.81
                    item_pose.position.y = -0.24
                    item_pose.position.z = 0.203
                    item_pose.orientation.x = 0.08
                    item_pose.orientation.y = 0.98
                    item_pose.orientation.z = -0.013
                    item_pose.orientation.w = 0.134
                    pnp.place(item_pose)
                    
                else:
                    val = itemposes[key]
                    itemloc = val.pose
                    item_loc_x.append(itemloc.position.x)
                    item_loc_y.append(itemloc.position.y)
                    item_loc_z.append(itemloc.position.z)
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
            for i in xrange(len(item_loc_x)): 
                block_pose = Pose()
                block_pose.position.x = item_loc_x[i]
                block_pose.position.y = item_loc_y[i]
                block_pose.position.z = item_loc_z[i]
                block_pose.orientation.x = 0.0
                block_pose.orientation.y = 0.0
                block_pose.orientation.z = 0.0
                block_pose.orientation.w = 1.0
                pnp.pick(block_pose)
                item_pose = Pose()
                if count > 5:
                    item_pose.position.x = 1.100
                    item_pose.position.y = -0.4
                    item_pose.position.z = 0.25
                    item_pose.orientation.x = 0.098
                    item_pose.orientation.y = 0.96
                    item_pose.orientation.z = -0.065
                    item_pose.orientation.w = 0.27
                else:
                    item_pose.position.x = 0.85
                    item_pose.position.y = -0.44
                    item_pose.position.z = 0.28
                    item_pose.orientation.x = 0.0968
                    item_pose.orientation.y = 0.96
                    item_pose.orientation.z = -0.06
                    item_pose.orientation.w = 0.27
                pnp.place(item_pose)
                count+= 1
    
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
    else:
        print "try again"

if __name__ == '__main__':
    main(sys.argv)