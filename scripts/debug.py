#!/usr/bin/env python

"""
Created on Sun Dec 17 19:16:17 2017

@author: lab
"""
import baxter_interface
import rospy
import time

rospy.init_node("debugmode", anonymous=True)
limb=baxter_interface.Limb("right")
limb.set_joint_position_speed(1.0)
rate = rospy.Rate(1000)
start_time = time.time()
if time.time() - start_time < 0.5:
    firstAngles = {'right_w0':-0.816,
               'right_w1':0.357,
               'right_w2':0.744,
               'right_e0':0.211,
               'right_e1':0.85,
               'right_s0':0.530,
               'right_s1':-0.734}
    limb.set_joint_positions(firstAngles)
if time.time() -start_time > 0.5 and time.time() - start_time > 3:
    secondAngles = {'right_w0':-0.83,
               'right_w1':-0.0314,
               'right_w2':0.849,
               'right_e0':-0.112,
               'right_e1':0.677,
               'right_s0':1.219,
               'right_s1':-0.667}
    limb.set_joint_positions(secondAngles)
if time.time() - start_time >3 and time.time() - start_time<8:
    thirdAngles = {'right_w0':-0.67,
               'right_w1':1.017,
               'right_w2':0.504,
               'right_e0':1.176,
               'right_e1':1.926,
               'right_s0':0.0913,
               'right_s1':-0.997}
    limb.set_joint_positions(thirdAngles)

