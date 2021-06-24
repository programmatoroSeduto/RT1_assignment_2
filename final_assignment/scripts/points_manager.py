#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse
import random



# server 'get_point'
name_get_point = "/get_point"

# server 'position_defined'
name_position_defined = "/position_defined"



# Available positions
positions = [ Point(-4,-3,0), Point(-4,2,0), Point(-4,7,0), Point(5,-7, 0), Point(5,-3,0), Point(5,1,0) ]




''' get one point randomly
	
	\param req [final_assignment/get_pointRequest] empty. 
'''
def cbk_get_point( req ):
	global positions
	
	to_return = get_pointResponse( )
	to_return.position = random.choice( positions )
	
	return to_return



''' check is one given point is defined
	
	\param req [final_assignment/position_definedRequest] the point
'''
def cbk_point_defined( req ):
	global positions
	
	to_return = position_definedResponse( )
	to_return.defined = True if ( req.position in positions ) else False
	
	return to_return



''' shutdown message

'''
def shut_msg():
	rospy.loginfo( " [points_manager] shutdown" )



''' node main.

'''
if __name__ == "__main__":
	rospy.init_node( "points_manager" )
	
	# advertise the server 'get_point'
	handler_get_point = rospy.Service( name_get_point, get_point, cbk_get_point )
	rospy.loginfo( " [points_manager] %s ONLINE.", name_get_point )
	
	# advertise the server 'psition_defined'
	handler_position_defined = rospy.Service( name_position_defined, position_defined, cbk_point_defined )
	rospy.loginfo( " [points_manager] %s ONLINE.", name_position_defined )
	
	rospy.on_shutdown( shut_msg )
	rospy.spin()
