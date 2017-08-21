#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from detectobjectusingtags import SceneObstacles

def main():
    print('========= MoveIt tutorial ==========')
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('movegroup_tutorial_python',anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")


    so = SceneObstacles()
#    so.addTrashAsObstacles()
    so.moveTrashIntoTrashcan()

    # print type(group)

    # display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    # print "......waiting for RVIZ..."
    # rospy.sleep(5)
    # print "......staring tutorial "

    # print "Reference frame: %s" % group.get_planning_frame()
    # print "End effector link: %s" % group.get_end_effector_link()
    # group_joint_values = group.get_current_joint_values()
    # print "Group joints values: ", group_joint_values
    # print "Robot groups: %s" % robot.get_group_names()
    # print "------- printing robot state:"
    # print robot.get_current_state()
    # print "------------------------------"

    # print "------Generating plan 1"
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.x = 1.0
    # pose_target.position.x = 0.7
    # pose_target.position.y = -0.05
    # pose_target.position.z = 1.1
    # group.set_pose_target(pose_target)
    # plan1 = group.plan()
    # print "(waiting while RVIZ displays plan1)"
    # rospy.sleep(5)
    # print "-----------------------"

    # group.clear_pose_targets()
    group_joint_values = group.get_current_joint_values()
    print "Current group joints values: ", group_joint_values
    print "Current group joints names: ", group.get_joints()

    # sys.exit()

    # group_joint_values[0] = 0.3
    # # print group.get_joints()[0]
    # print "joint {} moves to {}".format(group.get_joints()[0], group_joint_values[0])
    #
    # group.set_joint_value_target(group_joint_values)
    # plan2 = group.plan()
    # print "(waiting while RVIZ displays plan2)"
    # rospy.sleep(5)
    # print "-----------------------"
    # # Uncomment below line when working with a real robot
    # group.go(wait=True)

    group.clear_pose_targets()
    group.clear_path_constraints()
    waypoints = []
    group_current_pose = geometry_msgs.msg.Pose()
    try:
        group_current_pose = group.get_current_pose().pose
        print ("group_current_pose: ", group_current_pose)
    except MoveItCommanderException as e:
        print ("No end effector: ", e)
    except:
        print ("Unexpecte error while creating a trajectory")
    else:
        for i in range(1,2):
            # group.clear_pose_targets()
            # if i==1:
            #     group_current_pose.position.x += 0.1
            # else:
            #     group_current_pose.position.x -= 0.1
            # group.set_pose_target(group_current_pose)
            # group.plan()

            group_current_pose.position.x += 0.3
            waypoints.append(copy.deepcopy(group_current_pose))

            group_current_pose.position.y -= 0.3
            waypoints.append(copy.deepcopy(group_current_pose))

            group_current_pose.position.x -= 0.3
            waypoints.append(copy.deepcopy(group_current_pose))

            group_current_pose.position.y += 0.3
            waypoints.append(copy.deepcopy(group_current_pose))

            print('trajectory defined: ', waypoints)
            (plan3, fraction) = group.compute_cartesian_path(
                waypoints,
                0.01,
                0.0)
            print "(Waiting while RVIZ displays plan3)"
            rospy.sleep(5)
            print "-----------------------"
            print "TRAJECTORY POINTS NUMBER: ", len(plan3.joint_trajectory.points)
            # for p in plan3.joint_trajectory.points:
            #     p.velocities = [1*v for v in p.velocities]
            #     print p.velocities, "\n"
            print "FRACTION: ", fraction

            if not plan3.joint_trajectory.points:
                print ("No cartesian path found")
            elif fraction!=1.0:
                print ("Fraction is not 1")
            else:
                group.execute(plan3)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.logerr("testing python ros logging")
    main()
