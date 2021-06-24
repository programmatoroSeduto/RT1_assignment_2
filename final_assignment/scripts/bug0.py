#! /usr/bin/env python

import rospy
import time
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import bug0_status, bug0_statusRequest, bug0_statusResponse
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse

import math

node_name = "bug0"

## name of the service 'bug0_switch'
name_bug0_switch = "/bug0_switch"

## name of the service 'bug0_status'
name_bug0_status = "/bug0_status"

## name of the service 'get_point'
name_get_point = "/get_point"

## call-point of the service 'get_point'
srv_get_point = None

service_active = False
only_once = False
# TODO un timeout

pub = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None

yaw_ = 0

position_ = Point()
desired_position_ = Point()

regions_ = None

state_desc_ = ['Go to point', 'wall following', 'target reached']
state_ = 0
# 0 - go to point
# 1 - wall following

## err_pos AKA distance
err_pos = -1



def new_random_target( ):
	'''
		get a new random target from the service '/get_point'
	'''
	global name_get_point, srv_get_point
	global desired_position_
	
	# get the target from the server
	desired_position_ = ( srv_get_point( ) ).position
	
	# then write it on the parameter server
	rospy.set_param("des_pos_x", desired_position_.x)
	rospy.set_param("des_pos_y", desired_position_.y)




def srv_bug0_switch( data ):
	global service_active, only_once
	
	if data.val:
		if service_active:
			return switch_serviceResponse( success=False, in_progress=False )
		
		state_ = 2
		only_once = data.only_once
		
		# set the new target
		if not only_once:
			new_random_target( )
		
		service_active = True
	else:
		if not service_active:
			return switch_serviceResponse( success=True, in_progress=False )
		
		# the node is turned off
		service_active = False
		only_once = False
		
		# stop the robot
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
		
		# initial state
		state_ = 2
	
	return switch_serviceResponse( success=True, in_progress=False )



def srv_bug0_status( empty ):
	'''
	retrieve and return the status of the node
	'''
	global desired_position_, position_, yaw_, state_, err_pos
	
	to_return = bug0_statusResponse( )
	to_return.target_position = desired_position_
	to_return.actual_position = position_
	to_return.yaw = yaw_
	to_return.status = state_
	to_return.distance = err_pos
	
	return to_return




def clbk_odom(msg):
	global position_, yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (\
		msg.pose.pose.orientation.x,\
		msg.pose.pose.orientation.y,\
		msg.pose.pose.orientation.z,\
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def clbk_laser(msg):
	global regions_
	regions_ = {\
		'right':  min(min(msg.ranges[0:143]), 10),\
		'fright': min(min(msg.ranges[144:287]), 10),\
		'front':  min(min(msg.ranges[288:431]), 10),\
		'fleft':  min(min(msg.ranges[432:575]), 10),\
		'left':   min(min(msg.ranges[576:719]), 10),\
	}


def change_state(state):
	global state_, state_desc_, only_once, service_active
	global srv_client_wall_follower_, srv_client_go_to_point_
	state_ = state
	log = "state changed: %s" % state_desc_[state]
	rospy.loginfo(log)
	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)
	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)
	if state_ == 2:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
		
		if not only_once:
			new_random_target( )
		else:
			service_active = False
			only_once = False


def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def main():
	time.sleep(2)
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
	global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub
	global service_active, err_pos, only_once, state_
	
	rospy.init_node( node_name )
	
	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	srv_client_go_to_point_ = rospy.ServiceProxy( '/go_to_point_switch', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy( '/wall_follower_switch', SetBool)
	
	# initialize going to the point
	#change_state(0)
	state_ = 2
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if regions_ == None:
			continue
		
		if not service_active:
			continue
		
		if state_ == 0:
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			if(err_pos < 0.3):
				change_state(2)
				
			elif regions_['front'] < 0.5:
				change_state(1)
			
			# TODO incremento del timeout e possibile terminazione

		elif state_ == 1:
			desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
			err_yaw = normalize_angle(desired_yaw - yaw_)
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			
			if(err_pos < 0.3):
				change_state(2)
				
			if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
				change_state(0)
			
			# TODO incremento del timeout e possibile terminazione se viene sforato
			
		elif state_ == 2:
			# TODO reset del timeout a zero
			
			desired_position_.x = rospy.get_param('des_pos_x')
			desired_position_.y = rospy.get_param('des_pos_y')
			
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			
			if(err_pos > 0.35):
				change_state(0)
		
		rate.sleep()


if __name__ == "__main__":
	
	# service 'bug0_switch'
	rospy.loginfo( " [%s] advertising service %s ...", node_name, name_bug0_switch )
	rospy.Service( name_bug0_switch, switch_service, srv_bug0_switch )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_switch )
	
	# service 'bug0_status'
	rospy.loginfo( " [%s] advertising service %s ...", node_name, name_bug0_status )
	rospy.Service( name_bug0_status, bug0_status, srv_bug0_status )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_status )
	
	# service 'get_point'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_get_point )
	rospy.wait_for_service( name_get_point )
	srv_get_point = rospy.ServiceProxy( name_get_point, get_point )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_get_point )
	
	main()
