#! /usr/bin/env python

##
#	@file points_manager.py
#	@author Francesco Ganci (S4143910)
#	@brief Services for random target and default positions. 
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#		This simple ROS node provides 2 services:
#       <ul>
#       <li>\ref DOCSRV_get_point "/get_point" : it returns a randomly choosen target amond the predefined targets. </li>
#       <li>\ref DOCSRV_position_defined "/position_defined" : it returns true if the sent point belongs to the set of predefined ones. </li>
#       </ul>
#	
#	@copyright Copyright (c) 2021
#

import rospy
from geometry_msgs.msg import Point
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse
import random
from datetime import datetime



# --------------------------------- DATA

## name of this node
node_name = "points_manager"

## Available positions
positions = [ Point(-4,-3,0), Point(-4,2,0), Point(-4,7,0), Point(5,-7, 0), Point(5,-3,0), Point(5,1,0) ]



# --------------------------------- SERVICES

# server 'get_point'
name_get_point = "/get_point"

# server 'position_defined'
name_position_defined = "/position_defined"



## get one point randomly
#	
#	\param req [final_assignment/get_pointRequest] empty. 
#
def cbk_get_point( req ):
	global positions
	
	to_return = get_pointResponse( )
	to_return.position = random.choice( positions )
	
	return to_return



## check is one given point is defined
#	
#	\param req [final_assignment/position_definedRequest] the point
#
def cbk_point_defined( req ):
	global positions
	
	to_return = position_definedResponse( )
	to_return.defined = True if ( req.position in positions ) else False
	
	return to_return



# --------------------------------- NODE

## shutdown message
def shut_msg():
	rospy.loginfo( " [points_manager] shutdown" )



if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutdown )
	
	random.seed( datetime.now( ) )
	
	# advertise the server 'get_point'
	handler_get_point = rospy.Service( name_get_point, get_point, cbk_get_point )
	rospy.loginfo( " [points_manager] %s ONLINE.", name_get_point )
	
	# advertise the server 'psition_defined'
	handler_position_defined = rospy.Service( name_position_defined, position_defined, cbk_point_defined )
	rospy.loginfo( " [points_manager] %s ONLINE.", name_position_defined )
	
	rospy.on_shutdown( shut_msg )
	rospy.spin()
