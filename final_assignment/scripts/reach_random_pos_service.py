
#! usr/bin/env python 

import rospy
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse
from geometry_msgs import Point

# --------------------------------- DATA

## name of this node
node_name = "reach_random_pos_service"


## name of the service 'reach_random_pos_status'
name_reach_random_pos_switch = "/reach_random_pos_switch"

## name of the service 'check_position'
name_check_position = "/check_position"

## status of the node (bool; default: False)
service_active = False

## last pos signal received (bool; default: False)
signal_last_pos = False

## the robot is moving (bool; default: False)
is_moving = False

## actual position of the robot (geometry_msgs/Point)
actual_position = Point( )

## response from the service 'check_position' (final_assignment/check_position)
last_response_check_pos = check_positionResponse( )

## the target position (geometry_msgs/Point)
target_position = Point( )

## tolerance on the distance from the target
min_distance_from_the_target = 2

## time between two cycles
cycle_time = rospy.Duration( 0, 500 )




# --------------------------------- FUNCTIONS

def update_current_position( ):
	'''
	Update the informations about the position from check_position service.
	if the robot is_moving, make a common request,
	else, do nothing.
	'''
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



# --------------------------------- SERVICES

## call-point of the service 'check_position'
srv_check_position = None



def srv_reach_random_pos_switch( data ):
	''' Turn on and off the service. 
		
		\param data (final_assignment/switch_service)
	'''
	global service_active
	
	if data.val:
		''' TODO
			if the service is active, 
				signal_last_pos false
				return succcess
			else
				activate the service
				return success
		'''
		pass
	else:
		'''TODO
			if the service is active, 
				signal_last_pos true
				if the robot is moving, return not success, in progress
		'''
		pass



# --------------------------------- TOPICS

## ...



# --------------------------------- WORKING CYCLE

## ...



# --------------------------------- NODE

def cbk_on_shutdown():
	'''
	This is called on the shutdown of the node. 
	'''
	rospy.loginfo( " [%s] closing...", node_name )



def main():
	'''
	some word about the main function.
	'''
	global service_active, cycle_time
	
	while not rospy.is_shutdown( ):
		# update the current position
		update_current_position( )
		
		if service_active:
			# TODO update depending on the state of the robot 
			pass
		
		rospy.sleep( cycle_time )



if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutwown )
	
	# require the server 'check_position'
	rospy.loginfo( " [%] advertising service %s ...", node_name, name_reach_random_pos_status )
	rospy.Service( name_reach_random_pos_status, reach_random_pos_status, srv_reach_random_pos_status )
	rospy.loginfo( " [%] service %s ... OK", node_name, name_reach_random_pos_status )
	
	# service 'check_position'
	rospy.loginfo( " [%] getting service %s ...", node_name, name_check_position )
	rospy.wait_for_service( name_check_position )
	srv_check_position = rospy.ServiceProxy( name_check_position, check_position )
	rospy.loginfo( " [%] service %s ... OK", node_name, name_check_position )
	
	try:
		main()
	except rospy.ROSException:
		# ... properly manage the exception
		# https://docs.python.org/3/tutorial/errors.html
		rospy.loginfo( " [user console] Raised an Exception ... " )
