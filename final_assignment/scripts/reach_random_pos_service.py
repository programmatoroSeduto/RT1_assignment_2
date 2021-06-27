#! /usr/bin/env python 

##
#	@file reach_random_pos_service.py
#	@author Francesco Ganci (S4143910)
#	@brief Background process: ask for a target, then reach it using <i>move_base</i>
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#		This ROS node implements the move_base version of the functionality [no.1] <b>reach_random_pos_service</b>, executed in background. <br>
#       The interface is made up of 2 services: 
#       <ul>
#       <li> \ref DOCSRV_reach_random_pos_switch "/reach_random_pos_switch" : turn on and off the service. </li>
#       <li> \ref DOCSRV_reach_random_pos_status "/reach_random_pos_status" : inspect the progress of the motion towards the goal, and other informations. </li>
#       </ul>
#       When the node is active, it ciclically asks for a target to \ref DOCSRV_get_point "/get_point" service, then sends to <i>move_base</i> the goal and reached is. <i>move_base</i> is not interruptable in this implementation, so when the <i>stop</i> command is raised, the command is marked as <i>in progress</i> until the robot has reached the last set target. <br>
#       This node is implemented as a parallel process: services are always non-blocking, so the caller process can do other things while the robot is moving. <br>
#	
#	<br>
#	
#   <b>See also:</b>
#       <ul>
#       <li> \ref user_console.py "User Console Component" </li>
#       <li> \ref DOCSRV_reach_random_pos_switch "/reach_random_pos_switch service" </li>
#       <li> \ref DOCSRV_reach_random_pos_status "/reach_random_pos_status service" </li>
#       <li> \ref bug0.py "bug0", an alternative to move_base with a very similar interface. </li>
#       </ul>
#		
#	@copyright Copyright (c) 2021
#



import rospy
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse
from final_assignment.srv import reach_random_pos_status, reach_random_pos_statusRequest, reach_random_pos_statusResponse
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point



# --------------------------------- DATA

## name of this node
node_name = "reach_random_pos_service"


## name of the service 'reach_random_pos_status'
name_reach_random_pos_status = "/reach_random_pos_status"


## name of the service 'reach_random_pos_switch'
name_reach_random_pos_switch = "/reach_random_pos_switch"


## name of the service 'check_position'
name_check_position = "/check_position"


## name of the service 'get_point'
name_get_point = "/get_point"


## name of the topic 'move_base/goal' (Publisher)
name_move_base = "/move_base/goal"

## handler topic 'move_base/goal' (Publisher)
topic_move_base = None


## status of the node (bool; default: False)
service_active = False

## the robot is moving (bool; default: False)
is_moving = False

## last pos signal received (bool; default: False)
signal_last_pos = False


## actual position of the robot (geometry_msgs/Point)
actual_position = Point( )

## the target position (geometry_msgs/Point; default: None)
target_position = None


## response from the service 'check_position' (final_assignment/check_position)
last_response_check_pos = check_positionResponse( )


## tolerance on the distance from the target
min_distance_from_the_target = 0.5

## time between two cycles
cycle_time = rospy.Duration( 0, 500 )




# --------------------------------- FUNCTIONS

##
#   \brief Update the informations about the position from check_position service.
#   
#   \details
#       if the robot is_moving, make a common request to \ref DOCSRV_check_position "/check_position" service; else, do nothing.
#
def update_current_position( ):
	global is_moving, min_distance_from_the_target, target_position, last_response_check_pos, actual_position
	global srv_check_position
	
	if is_moving:
		# require an update
		req = check_positionRequest( )
		
		req.check_only = False
		req.tol = min_distance_from_the_target
		req.target = target_position

		res = srv_check_position( req )
		
		# store the last message 
		last_response_check_pos = res
		
		# store the last position
		actual_position = res.actual_position



##  \brief clear the status of the node.
#   
#   \details
#       This is called when the service is turned off. Simple setting of booleans to 'off' state. <br>
#       (not is_moving), (not signal_last_pos), (not target_pos)
#   
def clear_status( ):
	global is_moving, signal_last_pos, target_position
	
	is_moving = False
	signal_last_pos = False
	target_position = None
	
	pass



##
#   \brief ask a new randomly-choosen target to the server points_manager, then send to MoveBase the target to reach . 
#   
def new_target_to_move_base( ):
	global name_get_point, srv_get_point
	global name_move_base, srv_move_base
	global target_position
	
	# ge the new target
	target_position = ( srv_get_point( ) ).position
	
	# send it to move_base
	msg = MoveBaseActionGoal( )
	msg.goal.target_pose.header.frame_id = 'map'
	msg.goal.target_pose.pose.position.x = target_position.x
	msg.goal.target_pose.pose.position.y = target_position.y
	msg.goal.target_pose.pose.orientation.w = 1.0

	topic_move_base.publish(msg)



# --------------------------------- SERVICES

## call-point of the service 'check_position'
srv_check_position = None

## call-point of the service 'get_point'
srv_get_point = None


##
#   \brief Turn on and off the service. 
#   
#   \param data (final_assignment/switch_service) the switch. 
#   
#   \details
#       The switch has this behaviour:
#       <ul>
#       <li> Any 'turn-on' signal when the service is already active will cause the cancellation of any last_pos signal previously raised. </li> 
#       <li> If the 'turn-on' signal is received and the service is off, the request always succeeds, and the service is turned on. </li>
#       <li> Any 'turn-off' signal when the node is turned off results in a not-success response </li>
#       <li> The 'turn-off' request isn't immediately executed, because in this implementation move_base is not interruptible. The request is marked as <i>in_progress</i>, and resolved when the robot arrives at its destination. </li>
#       </ul>
#   
def srv_reach_random_pos_switch( data ):
	global service_active, signal_last_pos, is_moving
	
	response = switch_serviceResponse()
	
	if data.val:
		# i want to turn on the service
		if service_active:
			# turn off any last pos signals
			signal_last_pos = False
		else:
			# activate the service
			service_active = True
		
		# in any case, return this:
		response.success = True
		response.in_progress = False
		
	else:
		# i want to turn off the service
		if service_active:
			if is_moving:
				# signal last pos
				signal_last_pos = True
				
				# request in progress
				response.in_progress = True
				response.success = False
			else:
				# clear the status and turn off
				#    already done from the while cycle
				response.success = True
				response.in_progress = False
		else:
			# you're trying to turn off an already turned off service
			#    the server was turned off
			response.success = True
			response.in_progress = False
	
	return response



## \brief return the status of the service. 
def srv_reach_random_pos_status( data ):
	global service_active, actual_position, target_position, last_response_check_pos, signal_last_pos
	
	msg = reach_random_pos_statusResponse( )
	
	msg.is_active = service_active
	
	if target_position is not None:
		msg.target = target_position
	else:
		msg.target = Point()
	
	msg.distance = last_response_check_pos.distance 
	msg.actual_position = actual_position
	msg.last_pos_signal = signal_last_pos
	
	return msg



# --------------------------------- WORKING CYCLE

## 
#	@brief The algorithm of the node. 
#	
#	\details
#		This 'main' function contains the working algorithm of the node. Its functioning can be summarized as follows: <br>
#       <ul>
#       <li> if the robot is moving, check the position ad evaluate the distance. If the distance is under the tolerance, set the is_moving flag to false. </li> 
#       <li> If the robot is not moving, but no last_pos_signal is raised, gather a new position and set is_moving true; the cycle begins again. </li> 
#       <li> If the robot is not moving and the signal_last_pos is on, turn off the node. </li> 
#       </ul>
#
def main():
	global service_active, cycle_time, target_position, is_moving
	global name_get_point, srv_get_point
	global last_response_check_pos, signal_last_pos
	
	
	while not rospy.is_shutdown( ):
		
		# update the current position
		update_current_position( )
		
		if not service_active:
			# do nothing
			rospy.sleep( cycle_time )
			continue
		
		if not is_moving:
			if signal_last_pos:
				# last pos signal received
				#   target reached
				#   clear the node and deactivate it
				clear_status( )
				
				service_active = False
			else:
				# need for a new target
				new_target_to_move_base()
				
				# the robot starts to move
				is_moving = True
				
		else:
			# check the progress towards the target
			if last_response_check_pos.reached:
				# stop the movement
				is_moving = False
			else:
				# do nothing
				pass
		
		rospy.sleep( cycle_time )



# --------------------------------- NODE

## This is called on the shutdown of the node. 
def cbk_on_shutdown():
	rospy.loginfo( " [%s] is OFFLINE", node_name )




if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutdown )
	
	# require the server 'reach_random_pos_status'
	rospy.loginfo( " [%s] advertising service %s ...", node_name, name_reach_random_pos_status )
	rospy.Service( name_reach_random_pos_status, reach_random_pos_status, srv_reach_random_pos_status )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_status )
	
	# service 'check_position'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_check_position )
	rospy.wait_for_service( name_check_position )
	srv_check_position = rospy.ServiceProxy( name_check_position, check_position )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_check_position )
	
	# service 'get_point'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_get_point )
	rospy.wait_for_service( name_get_point )
	srv_get_point = rospy.ServiceProxy( name_get_point, get_point )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_get_point )
	
	# require the topic 'move_base/goal'
	rospy.loginfo( " [%s] topic (out) %s ...", node_name, name_move_base )
	topic_move_base = rospy.Publisher( name_move_base, MoveBaseActionGoal, queue_size=1 )
	rospy.loginfo( " [%s] topic (out) %s ... OK", node_name, name_move_base )
	
	# service 'reach_random_pos_switch'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_reach_random_pos_switch )
	rospy.Service( name_reach_random_pos_switch, switch_service, srv_reach_random_pos_switch )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_switch )
	
	try:
		rospy.loginfo( " [%s] is ONLINE", node_name )
		main()
	except rospy.ROSException:
		rospy.loginfo( " [%s] Raised an Exception ... ", node_name )
