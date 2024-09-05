#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray

hola_x = 0
hola_y = 0
hola_theta = 0

x_goals = [0,1,-1,0,0]
y_goals = [1,-1,-1,1,-1]
theta_goals = [0, 0, 0, 0, 0]

def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def odometryCb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	qrt=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
	eul=euler_from_quaternion(qrt)
	hola_theta = eul[2]
	if(hola_theta>=360) or hola_theta<=-360:
		hola_theta=hola_theta-360
	elif(hola_theta<=-360):
		hola_theta=360+hola_theta

def main():
	rospy.init_node('controller',anonymous=True)
	pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
	rospy.Subscriber( '/odom', Odometry, callback = odometryCb )
	rospy.Subscriber('task1_goals', PoseArray, callback = task1_goals_Cb)
	vel = Twist()
	rate = rospy.Rate(100)
	index = 0
	x_vel = 0 
	y_vel = 0

	while not rospy.is_shutdown():
		max_vel = 1
		x_e = x_goals[index]-hola_x
		y_e = y_goals[index]-hola_y
		theta_e = theta_goals[index]-hola_theta
		if(theta_e<=0.01 and theta_e>=-0.01 and x_e<=.01 and x_e>=-.01 and y_e<=.01 and y_e>=-.01):
			
			if(index<len(x_goals)-1):
				index+=1
			else:
				index = 0


		else:
			if(x_e<=max_vel and x_e>=-max_vel):
				x_vel = x_e
			elif(x_e>=max_vel):
				x_vel = max_vel
			elif(x_e<=-max_vel):
				x_vel = -max_vel
			if(y_e<=max_vel and y_e>=-max_vel):
				y_vel = y_e
			elif(y_e>=max_vel):
				y_vel = max_vel
			elif(y_e<=-max_vel):
				y_vel = -max_vel
			vel.linear.x = x_vel*math.cos(hola_theta)+y_vel*math.sin(hola_theta)
			vel.linear.y = y_vel*math.cos(hola_theta)-x_vel*math.sin(hola_theta)
			vel.angular.z=theta_e*3
			pub.publish(vel)
	rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

