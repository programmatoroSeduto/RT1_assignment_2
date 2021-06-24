#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse

name_odom = "/odom"
topic_odom = None

name_check_position = "/check_position"

# position, orientation
actual_pose = Pose()


def get_odom_from_topic( data ):
	global actual_pose
	actual_pose = data.pose.pose



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


if __name__ == "__main__":
	rospy.init_node( "check_position" )
	
	rospy.loginfo( " [check_position] topic (sub) %s ...", name_odom )
	topic_odom = rospy.Subscriber( name_odom, Odometry, get_odom_from_topic )
	rospy.loginfo( " [check_position] topic (sub) %s ... OK", name_odom )
	
	rospy.loginfo( " [check_position] advertising %s ...", name_check_position )
	rospy.Service( name_check_position, check_position, srv_check_position )
	rospy.loginfo( " [check_position] advertising %s ... OK", name_check_position )
	
	rospy.spin()
