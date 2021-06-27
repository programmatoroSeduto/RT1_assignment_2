#! /usr/bin/env python

##
#	@file bug0.py
#	@authors Prof. <b>Carmine Recchiuto</b> (first version), <i>Francesco Ganci</i> (S4143910) (Service version)
#	@brief A finite sate machine for driving a mobile robot towards a target. 
#	@version 2.0
#	@date 2021-06-25
#	
#	\details
#   The node implements a simple planning algorithm, an alternative to <i>move_base</i>. It works under the assumptions that a robot has a good odometry (see SLAM_gmapping) and has laser sensors. <br>
#   Formally speaking, the node is a finite state machine, reacting to the sensor and to the distance from a given target:
#   <ul>
#   <li><b> State 0</b> : the robot goes straight until it finds a wall </li>
#   <li><b> State 1</b> : the robot follows the wall, see wall_follow_service_m.py </li>
#   <li><b> State 2</b> : the robot has "reached" the target (i.e. the robot is close enough to the target position) </li>
#   </ul>
#   The node can be turned on and off, exposing a service interface quite similar (architecturally speaking) to the node reach_random_pos_service.py .<br>
#   Two behaviours are implemented here:
#   <ul>
#   <li> ask for a random target, then try and reach it </li>
#   <li> given a target via parameter server, try and reach the target. </li>
#   </ul>
#   When turned off, the service immediately blocks the robot in the actual position. <br>
#   Note that this node doesn't directly publish the velocity to the simulated environment, rather it uses the nodes wall_follow_service_m.py and go_to_point_service_m.py for the low-level control. <br>
#	
#   @see wall_follow_service_m.py the wall follower component
#   @see go_to_point_service_m.py the go to point component
#   @see reach_random_pos_service.py an alternative service using move_base
#   
#   <b>Related services:</b>
#   <ul>
#   <li>\ref DOCSRV_bug0_switch "bug0 switch service"</li>
#   <li>\ref DOCSRV_bug0_status "bug0 status service"</li>
#   </ul>
#   
#	@copyright Copyright (c) 2021
#

import rospy
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import bug0_status, bug0_statusRequest, bug0_statusResponse
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse

import math

# --------------------------------- DATA

## name of this node
node_name = "bug0"

## name of the service 'bug0_switch'
name_bug0_switch = "/bug0_switch"

## name of the service 'bug0_status'
name_bug0_status = "/bug0_status"

## name of the service 'get_point'
name_get_point = "/get_point"

## call-point of the service 'get_point'
srv_get_point = None

## Is the service active? 
service_active = False

## Only_once mode (see \ref DOCSRV_bug0_switch "bug0_switch" service)
only_once = False

## handler for the topic 'cmd_vel' (Publisher)
pub = None

## Actual orientation wrt 'z' axis
yaw_ = 0

## actual position
position_ = Point()

## target position
desired_position_ = Point()

## Laser regions
regions_ = None

## Status labels
state_desc_ = ['Go to point', 'wall following', 'target reached']

## Actual status
#
#   \details
# 0 - go to point<br>
# 1 - wall following<br>
# 2 - goal reached 
# 
state_ = 0

## distance from the target
err_pos = -1

## Tolerance on the distance from the target
max_tolerance_target = 0.5


# --------------------------------- FUNCTIONS

## 
#	@brief Get a new randomly-chosen target
#	
#	\details
#		The only purpose of this function is requiring a new target from the \ref DOCSRV_get_point "/get_point" server, and storing it into the parameter server.
#	    See \ref DOCSRV_get_point "/get_point" .
#
def new_random_target( ):
	global name_get_point, srv_get_point
	global desired_position_
	
	# get the target from the server
	desired_position_ = ( srv_get_point( ) ).position
	
	# then write it on the parameter server
	rospy.set_param("des_pos_x", desired_position_.x)
	rospy.set_param("des_pos_y", desired_position_.y)



## 
#	@brief Change the state of the node. 
#	
#	@param state (Int) the next state. 
#	
#	\details
#   The function changes the state of the robot from the actual one to the one passed as argument. 
#   <ul>
#   <li> <b>To state 0</b> : only go_to_point, wall_follow is turned off </li>
#   <li> <b>To state 1</b> : only wall_follow, go_to_point is off </li>
#   <li> <b>To state 2</b> : the robot is stopped, and a new target is required if the node is not in 'only_once' mode.  </li>
#   </ul>
#
def change_state( state ):
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



## 
#	@brief Normalize an angle in [-pi, pi]
#	
#	@param angle (float) the angle, in radiants
#	@return float the normalised angle
#
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle



# --------------------------------- SERVICES

## call-point of the service 'go_to_point_switch'
srv_client_go_to_point_ = None

## call-point of the service 'wall_follow_switch'
srv_client_wall_follower_ = None



## 
#	@brief Turn on or off the node.
#	
#	@param data (final_assignment/switch_serviceRequest) The request
#	
#	\details
#   Here is how this switch works:
#   <ul>
#   <li> 
#   If the request is to <i>activate the service</i>:
#   <ul>
#   <li> not success if the service is already active </li>
#   <li> otherwise, activate the service and return success to the caller. </li>
#   </ul>
#   </li>
#   <li> 
#   If the request is to <i>stop the service</i>:
#   <ul>
#   <li> if the service is already turned off, return not succes.  </li>
#   <li> Otherwise, the robot is immediately stopped, and the server is goes off. Return success</li>
#   </ul>
#   </li>
#   </ul>
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



## 
#	@brief Get the status of the service
#	
#	@param empty (final_assignment/bug0_status) empty request
#	
#	\details
#       The function simply sends a message containing the most significant informations from the data section.  <br>
#       See \ref DOCSRV_bug0_status "/bug0_status" . 
#
def srv_bug0_status( empty ):
	global desired_position_, position_, yaw_, state_, err_pos, service_active, max_tolerance_target, err_pos
	
	to_return = bug0_statusResponse( )
	to_return.target_position = desired_position_
	to_return.actual_position = position_
	to_return.actual_yaw = yaw_
	to_return.status = state_
	to_return.active = service_active
	to_return.distance = err_pos
	to_return.reached = ( err_tolerance < max_tolerance_target )
	
	return to_return



# --------------------------------- TOPICS

## 
#	@brief Get the actual posture.
#	
#	@param msg (nsv_msgs/Odometry) the actual posture from the simulation.
#	
#	\details
#		The function reads and adapts the informations from the topic '/odom'.
#
def clbk_odom( msg ):
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



## 
#	@brief Get the laser measurements.
#	
#	@param msg (sensor_msgs/LaserScan) raw-update from the lasers.
#	
#	\details
#		The function reads and adapts the informations from the topic '/scan'. <br>
#       Used in managing states '1' and '2'
#
def clbk_laser(msg):
	global regions_
	regions_ = {\
		'right':  min(min(msg.ranges[0:143]), 10),\
		'fright': min(min(msg.ranges[144:287]), 10),\
		'front':  min(min(msg.ranges[288:431]), 10),\
		'fleft':  min(min(msg.ranges[432:575]), 10),\
		'left':   min(min(msg.ranges[576:719]), 10),\
	}



# --------------------------------- WORKING CYCLE

## 
#	@brief The main section of the Bug0 algorithm
#	
#	\details
#	The state machine works in this way:
#   <ul>
#   <li> <b>State 0</b> : <i>go straight</i>. If the robot has reached the target, go to state 2. If there is a wall opposite to the robot, go to state 1.  </li>
#   <li> <b>State 1</b> : <i>follow the wall</i>. If there is a way point, go to state 0. If the target is reached, go to state 2. </li>
#   <li> <b>State 2</b> : <i>read the new target from the parameter server</i>. If the robot is already in the goal position, go to state 2. Otherwise, go to state 0.  </li>
#   </ul>
#
def main():
	time.sleep(2)
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
	global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub
	global service_active, err_pos, only_once, state_, max_tolerance_target
	
	# initialize: get a new target
	state_ = 2
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if regions_ == None:
			continue
		
		if not service_active:
			continue
		
		if state_ == 0:
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			if(err_pos < max_tolerance_target):
				change_state(2)
				
			elif regions_['front'] < 0.5:
				change_state(1)
			
		
		elif state_ == 1:
			desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
			err_yaw = normalize_angle(desired_yaw - yaw_)
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			
			if(err_pos < max_tolerance_target):
				change_state(2)
				
			if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
				change_state(0)
			
			
		elif state_ == 2:
			
			desired_position_.x = rospy.get_param('des_pos_x')
			desired_position_.y = rospy.get_param('des_pos_y')
			
			err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
			
			if(err_pos > max_tolerance_target + 0.05):
				change_state(0)
		
		rate.sleep()



# --------------------------------- NODE

## Called on the shutdown of the node. 
def cbk_on_shutdown():
	global node_name
	
	rospy.loginfo( " [%s] is OFFLINE", node_name )




if __name__ == "__main__":
	
	rospy.init_node( node_name )
	
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
	
	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	srv_client_go_to_point_ = rospy.ServiceProxy( '/go_to_point_switch', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy( '/wall_follower_switch', SetBool)
	
	main()
