#!/usr/bin/env python
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import sys
import rospy
from geometry_msgs.msg import PoseStamped

if __name__=='__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous= True)
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("right_arm")
    right_gripper = MoveGroupCommander("right_gripper")
    rospy.sleep(1)
    scene.remove_world_object("table")
    right_arm.set_named_target("resting")
    right_arm.go()
    right_gripper.set_named_target("resting")
    right_arm.go()
    right_gripper.set_named_target("open")
    right_gripper.go()
    rospy.sleep(1)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.5
    p.pose.position.y = 0.5
    p.pose.position.z = 0.5
    scene.add_box("table",p,(0.5,0.5,0.5))
    
    rospy.spin()
    roscpp_shutdown()
 
