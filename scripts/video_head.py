#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image


def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        display_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node("my_cam")
	display_pub= rospy.Publisher('/robot/xdisplay',Image)

	right_camera = baxter_interface.CameraController("right_hand_camera")
	right_camera.close()
	head_camera = baxter_interface.CameraController("head_camera")

	head_camera.resolution =(1280, 800)
	head_camera.open()

	left_camera = baxter_interface.CameraController("left_hand_camera")
	left_camera.resolution =(1280, 800)
	#left_camera.resolution =(960, 600)
	#left_camera.resolution =(640, 400)
	#left_camera.resolution =(480, 300)
	#left_camera.resolution =(320, 200)
	left_camera.open()

	camera_name = "head_camera"
	#camera_name = "left_hand_camera"
	sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,republish,None,1)
	print("viewing head camera on screen...")
	#rospy.spin()
rospy.sleep(8)