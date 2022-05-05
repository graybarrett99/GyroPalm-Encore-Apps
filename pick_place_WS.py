#
#    Robot Control through Gestures
#    Author: William LaPlant
#    Co Authors: Corbin Newhard, Austin Ford, Alex Chui, William LaPlant,
#    Richard Lopez, Jaylen Thomas, and Flevin Young
#
#    Created on: March 15, 2022
#
#    Last Modified on: March 31, 2022
#
#    Description: This script uses a color/depth camera to get the arm to find objects and pick them up.
#    For this demo, the arm is placed to the left of the camera facing outward. When the
#    end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'px100/base_link' frame, the AR
#    tag should be clearly visible to the camera. A small basket should be placed in front of the camera.
#    To begin, run 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=px100'
#    Then change to this directory and type 'python3 pick_place_WS.py'

import asyncio
import websocket
import ssl
import json
import time
import colorsys
import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface



############################################### Main Method #################################################
# The main method establishes a websocket with gyropalm's web socket server @ port 3200. The connection is
# confirmed in the terminal by printing the matching wearable ID and apiKey.

def main():
	ws = websocket.WebSocket(sslopt={"cert_reqs": ssl.CERT_NONE}) # identify the web socket
	ws.connect("wss://gyropalm.com:3200") # connect to gyropalm web socket server at port 3200

	welcomeMessage = ws.recv() # save result of receiving message into variable called welcomeMessage
	welcomeMessage = json.loads(welcomeMessage) # convert the result from json into a String
	print("\n%s\n" % welcomeMessage) # print the message received from the web socket, which asks for GyroPalm cridentials

	authorizationMessage = json.dumps({'action':"sub", 'wearableID':"gp10222509", 'apiKey':"10af1ab4263a472"}, sort_keys = True, indent = 4) # save the matching authorization message into variable called authorizationMessage
	print("%s\n" % authorizationMessage) # print authorization message
	ws.send(authorizationMessage) # send authorization message including action, wearableID and apiKey of wearable

	confirmationMessage = ws.recv() #receive confirmation message from GyroPalm web socket server
	confirmationMessage = json.loads(confirmationMessage) # convert the result from json into a String
	print("%s\n" % confirmationMessage) # print the message received from the web socket server, which confirms the connection
	
	# initialize the arm module along with the pointcloud and armtag modules
	bot = InterbotixManipulatorXS("px100", moving_time=1.5, accel_time=0.75) # initialize robot with id, moving time and acceleration time
	pcl = InterbotixPointCloudInterface() # initialize cluster clouds
	armtag = InterbotixArmTagInterface() # initialize position of the robot in camera

    # Set initial arm and gripper pose
	bot.arm.set_ee_pose_components(x=0.3, z=0.2)
	bot.gripper.open()

    # Get the ArmTag pose
	bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
	time.sleep(0.5)
	armtag.find_ref_to_arm_base_transform()
	bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # Get cluster positions
    # Sort clusters from max to min 'x' position w.r.t. the 'px100/base_link' frame
	success, clusters = pcl.get_cluster_positions(num_samples=1, ref_frame="px100/base_link", sort_axis="x", reverse=True)
	
	while(1):
    	# Receive message and print the message
		msg = ws.recv()
		msg = json.loads(msg)
		print("%s" % msg)

		if(msg["gp10222509"] == 'Home'): # if Home button is pressed on wearable, send robot to Home position
			bot.arm.go_to_home_pose()
			print("Home")
				
		if(msg["gp10222509"] == 'Sleep'): # if Sleep button is pressed on wearable, send robot to Sleep position
			bot.arm.go_to_sleep_pose()
			print("Sleep")
				
		if(msg["gp10222509"] == 'dflick'): # if downflick gesture is performed on wearable, send robot to Sleep position  and find and save new set of clusters viewed by camera
			bot.arm.go_to_sleep_pose()
			clusters.clear()
			success, clusters = pcl.get_cluster_positions(num_samples=1, ref_frame="px100/base_link", sort_axis="x", reverse=True)
			
		if(msg["gp10222509"] == 'uflick'): # if upflick gesture is performed on wearable, move robot to blocks and place blocks in box
			print("uflick")
			for cluster in clusters:
				x, y, z = cluster["position"]
				bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.1, pitch=0.5)
				bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
				bot.gripper.close()
				bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.1, pitch=0.5)
					
				bot.arm.set_ee_pose_components(x=-0.15, y=-0.2, z=z+0.15, pitch=0.5) #bring blocks to the box
				bot.arm.set_ee_pose_components(x=0.3, z=0.2)
				bot.gripper.open()
			bot.arm.go_to_sleep_pose()
			clusters.clear()
			
		if(msg["gp10222509"] == 'wave'): # if wave gesture is performed on the wearable, the robot will 'wave'
			bot.arm.go_to_home_pose()
			bot.arm.set_single_joint_position(joint_name="wrist_angle", position=0.7, moving_time=0.7)
			bot.arm.set_single_joint_position(joint_name="wrist_angle", position=0.1, moving_time=0.7)
			bot.arm.set_single_joint_position(joint_name="wrist_angle", position=0.7, moving_time=0.7)
			bot.arm.set_single_joint_position(joint_name="wrist_angle", position=0.1, moving_time=0.7)
			bot.arm.go_to_sleep_pose(moving_time=1)

############################################### color_compare function #################################################
# The color_compare function determines the color of each object using the Hue value in the HSV color space. The function
# takes an input of an array, called rgb.

def color_compare(rgb):
    r,g,b = [x/255.0 for x in rgb]
    h,s,v = colorsys.rgb_to_hsv(r,g,b)

    if h < 0.025: color = "red"
    elif 0.025 < h < 0.05: color = "orange"
    elif 0.1 < h < 0.15: color = "yellow"
    elif 0.3 < h < 0.4: color = "green"
    elif 0.55 < h < 0.65: color = "blue"
    elif 0.75 < h < 0.85: color = "purple"
    else: color = "unknown"
    return color
    
# The following lines run the main method

if __name__=='__main__':
    main()
