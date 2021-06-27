#! /usr/bin/env python

##
#	@file reach_user_pos_service.py
#	@author Francesco Ganci (S4143910)
#	@brief Reach a user-defined position using move_base motion planning algorithm
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#		This ROS node implements a simple component in charge of sending to move_base a goal given by the user. It is the implementation of the functionality no.2 when move_base is set. 
#	
#	<br>
#	
#   <b>See also:</b>
#       <ul>
#       <li> \ref user_console.py "User Console Component" </li>
#       <li> \ref DOCSRV_user_target "/user_target service" </li>
#       </ul>
#	
#	@copyright Copyright (c) 2021
#

import rospy
from final_assignment.srv import user_target, user_targetRequest, user_targetResponse
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse


# --------------------------------- DATA

## name of this node
node_name = "reach_user_pos_service"

## name of the topic 'move_base/goal' (Publisher)
name_move_base = "/move_base/goal"

# name of the service 'user_target'
name_user_target = "/user_target"

## handler topic 'move_base/goal' (Publisher)
topic_move_base = None

## name of the service 'check_position'
name_check_position = "/check_position"


## tolerance on the distance from the target
min_distance_from_the_target = 2

## time between two cycles
cycle_time = rospy.Duration( 0, 500 )



# --------------------------------- FUNCTIONS

##
#   \brief send the goal to MoveBase.
#   
#    \param target_position (geometry_msgs/Point)
#   
#    \details
#    ask a new randomly-choosen target to the server points_manager<br>
#    then send to MoveBase the target to reach 
#    
def new_target_to_move_base( target_position ):
	global node_name
	global name_move_base, srv_move_base
	
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


##  \brief implementation of the service '/user_target'
#   
#   \param data (final_assignment/user_targetRequest) the goal from the user
#   
#   \details
#       get from the user a target, then reach it.  <br>
#       <b>BLOCKING SERVER</b> : the server doesn't return until some error occurrs, or the target has been reached. 
#   
#   <b>See also</b> : \ref DOCSRV_user_target "/user_target service" How to use it.
#   
def srv_user_target( data ):
	global node_name, min_distance_from_the_target
	global name_check_position, srv_check_position
	
	response = user_targetResponse( )
	
	# send the target to Move_Base
	rospy.loginfo( " [%s] sending target to MoveBase ... ", node_name )
	new_target_to_move_base( data.target )
	
	# wait until the target is reached
	rospy.loginfo( " [%s] reaching the position... ", node_name )
	# get the actual position
	req = check_positionRequest( )
	req.check_only = False
	req.tol = min_distance_from_the_target
	req.target = data.target
	while not ( srv_check_position( req ) ).reached:
		rospy.sleep( cycle_time )
	
	rospy.loginfo( " [%s] position reached! ", node_name )
	
	response.success = True
	return response



# --------------------------------- WORKING CYCLE

## a simple spin. 
def main():
	rospy.spin()



# --------------------------------- NODE

## This is called on the shutdown of the node. 
def cbk_on_shutdown():
	global node_name
	
	rospy.loginfo( " [%s] is OFFLINE", node_name )



if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutdown )
	
	# require the server 'user_target'
	rospy.loginfo( " [%s] advertising service %s ...", node_name, name_user_target )
	rospy.Service( name_user_target, user_target, srv_user_target )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_user_target )
	
	# require the topic 'move_base/goal'
	rospy.loginfo( " [%s] topic (out) %s ...", node_name, name_move_base )
	topic_move_base = rospy.Publisher( name_move_base, MoveBaseActionGoal, queue_size=1 )
	rospy.loginfo( " [%s] topic (out) %s ... OK", node_name, name_move_base )
	
	# service 'check_position'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_check_position )
	rospy.wait_for_service( name_check_position )
	srv_check_position = rospy.ServiceProxy( name_check_position, check_position )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_check_position )
	
	try:
		rospy.loginfo( " [%s] is ONLINE", node_name )
		main()
	except rospy.ROSException:
		rospy.loginfo( " [%s] Raised an Exception ... ", node_name )
