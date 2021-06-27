#! /usr/bin/env python

##
#	@file check_position.py
#	@author Francesco Ganci (S4143910)
#	@brief Check the actual position of the robot using <i>/odom</i> topic. 
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#		This ROS node implements the service \ref DOCSRV_check_position "/check_position", which is capable of doing these things:
#       <ul>
#       <li><b>Check only</b> : the service will return only the position of the robot</li>
#       <li><b>Distance Calculation</b> : given a target, the server returns the distance from it.</li>
#       </ul>
#	
#   <b>See also:</b> \ref DOCSRV_check_position "/check_position service"
#	
#	@copyright Copyright (c) 2021
#

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse



# --------------------------------- DATA

## name of this node
node_name = "check_position"

## topic '/odom' as Subscriber
name_odom = "/odom"

## name of the service 'check_position'
name_check_position = "/check_position"

## position, orientation
actual_pose = Pose()



# --------------------------------- FUNCTIONS

## 
#	@brief Implementation of the service <i>check_position</i>
#	
#	@param data (final_assignment/check_positionRequest) the request from the caller
#	
#	\details
#		If the flag 'check_only' is true, only send position and other informations. The value of 'distance' will be -1, 'reached' will be false.
#		If a target is given, the service returns 'not success' response if the tolerance is less or equal than 0. 
#   
#   <b>See also:</b> \ref DOCSRV_check_position "/check_position service"
#   
def srv_check_position( data ):
	global actual_pose
	
	to_return = check_positionResponse( )
	
	if data.check_only:
		to_return.reached = False
		to_return.success = True
		to_return.actual_position = actual_pose.position
		to_return.distance = -1
	else:
		target = data.target
		if ( data.tol > 0 ):
			to_return.distance = ( ( target.x - actual_pose.position.x )**2 + ( target.y - actual_pose.position.y )**2 )
			to_return.reached = ( to_return.distance <= data.tol )
			to_return.success = True
			to_return.actual_position = actual_pose.position
		else:
			to_return.reached = False
			to_return.success = False
			to_return.actual_position = actual_pose.position
			to_return.distance = -1
	
	return to_return



# --------------------------------- SERVICES

## 
#	@brief Store the actual pose. 
#	
#	@param data (nav_msgs/Odometry) the actual pose. 
#
def get_odom_from_topic( data ):
	global actual_pose
	actual_pose = data.pose.pose



# --------------------------------- TOPICS

## handler topic '/odom' as Subscriber
topic_odom = None



# --------------------------------- NODE

if __name__ == "__main__":
	rospy.init_node( node_name )
	
	# topic '/odom' as Subscriber
	rospy.loginfo( " [check_position] topic (sub) %s ...", name_odom )
	topic_odom = rospy.Subscriber( name_odom, Odometry, get_odom_from_topic )
	rospy.loginfo( " [check_position] topic (sub) %s ... OK", name_odom )
	
	# service '/check_position'
	rospy.loginfo( " [check_position] advertising %s ...", name_check_position )
	rospy.Service( name_check_position, check_position, srv_check_position )
	rospy.loginfo( " [check_position] advertising %s ... OK", name_check_position )
	
	rospy.spin()
