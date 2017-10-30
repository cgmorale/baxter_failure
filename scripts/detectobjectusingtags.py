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
        self.newPose.pose.position.x -=0.03
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
    def __init__(self, limb):
        self.limb = baxter_interface.Limb(limb)
        self.retreatdistance = 0.10
        self.gripper = baxter_interface.Gripper(limb)
        self._ir_sensor = IRSensor()
        while "right" not in self._ir_sensor.distance:
            rospy.sleep(1)            
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
        
#        MoveBaxter.reset_cameras(MoveBaxter())
##        MoveBaxter.close_camera(MoveBaxter(),"head")
#        MoveBaxter.open_camera(MoveBaxter(),"right")
#        MoveBaxter.subscribe_to_camera(MoveBaxter(),"right")
        
        
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
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/angry.jpg')
            self.openGripperToThrow()
        
    def movetoend(self):
        limb=baxter_interface.Limb("right")
        limb.set_joint_position_speed(0.8)
        endAngles = {'right_w0':-0.85,
                       'right_w1':0.26,
                       'right_w2':0.76,
                       'right_e0':0.436,
                       'right_e1':0.388,
                       'right_s0':0.708,
                       'right_s1':-0.622}
        limb.move_to_joint_positions(endAngles)
    
    def openg(self):
        rospy.sleep(1.8)
        self.gripper.command_position(position=100.0, block =False)
        self.gripper.open()
        
    def openGripperToThrow(self):
        threading.Thread(target= self.movetoend).start()
        threading.Thread(target= self.openg).start()
            
    def place(self,pose):
        self.approachBag(pose)
        self.servo_to_bag(pose)
        MoveBaxter.openGripper(MoveBaxter())
        self.retract_from_bag()
        
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
    MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
    brs = IRSensor()
    
#    PickPlace.load_gazebo_models(PickPlace(limb))
#    rospy.on_shutdown(PickPlace.delete_gazebo_models(PickPlace(limb)))
    number = input("choose state:")
    if number ==1:
        x = 1
        count = 0
        while x ==1:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
            pnp = PickPlace(limb)
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
    
    elif number ==2:
        x = 1
        count = 0
        while x ==1:
            MoveBaxter.changeBaxterFace(MoveBaxter(),'/home/lab/Pictures/baxterhappy.jpg')
            pnp = PickPlace(limb)
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
                else: 
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