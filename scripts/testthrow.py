#!/usr/bin/env python

import baxter_interface
import rospy
import threading
import time 


def moveArm():
    limb = baxter_interface.Limb('right')
    startAngles = {'right_w0':-0.67,
                   'right_w1':1.03,
                   'right_w2':0.5,
                   'right_e0':1.19,
                   'right_e1':1.94,
                   'right_s0':0.08,
                   'right_s1':-0.93}
    limb.move_to_joint_positions(startAngles)
    wristmotion = {'right_w0':-0.67,
                   'right_w1':-0.991,
                   'right_w2':0.5,
                   'right_e0':1.19,
                   'right_e1':1.94,
                   'right_s0':0.08,
                   'right_s1':-0.93}
    limb.set_joint_velocities({'right_w0':3.0})
    limb.move_to_joint_positions(wristmotion)
#    midAngles = {'right_w0':0.208,
#                   'right_w1':0.327,
#                   'right_w2':-0.421,
#                   'right_e0':-0.328,
#                   'right_e1':2.011,
#                   'right_s0':0.969,
#                   'right_s1':-0.775}
#    limb.move_to_joint_positions(midAngles)
#    limb.set_joint_position_speed(0.6)
#    print limb.joint_velocities()
#    endAngles = {'right_w0':-0.85,
#                   'right_w1':0.26,
#                   'right_w2':0.76,
#                   'right_e0':0.436,
#                   'right_e1':0.388,
#                   'right_s0':0.708,
#                   'right_s1':-0.622}
#    limb.move_to_joint_positions(endAngles)
    
def openGripper():
    t1 = threading.Thread(target= moveArm)
    t1.start()
    time.sleep(2)
    #time.sleep(5.85)
    gripper = baxter_interface.Gripper('right')
    gripper.command_position(position=100.0, block =True,timeout=5.0)
    gripper.open()
    

def main():
    rospy.init_node("throwingtest")
#    gripper = baxter_interface.Gripper('right')
#    gripper.calibrate(block=True, timeout=5.0)
    openGripper()
    
    
if __name__ == '__main__':
    main()
