# -*- coding: utf-8 -*-
"""
Created on Sun Feb  4 11:16:01 2018

@author: lab
"""
import rospy
from geometry_msgs.msg import Pose
import baxter_interface
import time

def main():
    rospy.init_node('swing_arm')
    limb=baxter_interface.Limb("right")
    limb.set_joint_position_speed(1.0)
    rate = rospy.Rate(1000)
    start_time = time.time()
    fourthAngles = {'right_w0': 0.375,
                    'right_w1': -1.36,
                    'right_w2': 0.431,
                    'right_e0': 0.243,
                    'right_e1': -0.041,
                    'right_s0': -0.057,
                    'right_s1': -0.741}
    while not rospy.is_shutdown():
        limb.set_joint_positions(fourthAngles)
        if time.time()-start_time > 1.5:
            fourthAngles = {'right_w0': -0.662,
                            'right_w1': 1.02,
                            'right_w2': 0.499,
                            'right_e0': 1.184,
                            'right_e1': 1.927,
                            'right_s0': 0.0779,
                            'right_s1': -0.997}
        rate.sleep()

    
if __name__ =='__main__':
    main()