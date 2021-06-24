#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse
from move_base_msgs.msg import MoveBaseActionGoal
from std_srvs.srv import SetBool

node_name = "user_console"



## available commands
commands = [ "", \
  "reach_random_pos", \
  "reach_user_pos", \
  "wall_follow", \
  "last_pos", \
  "change_motion_planning_algorithm", \
  "help", \
  "exit" ]

## infos about each command
commands_info = [ "", \
  "periodically ask a position to the server, then try and reach it.", \
  "ask to the user a position, then try and reach it. ", \
  "use the component wall_follower.", \
  "Stop any ongoing movement", \
  "select your motion planning algorithm between 'move_base' and 'bug0'", 
  "print this help", 
  "Close this program." ]

# server 'get_point'
name_get_point = "/get_point"
srv_get_point = None

# server 'check_position'
name_check_position = "/check_position"
srv_check_position = None

# server 'position_defined'
name_position_defined = "/position_defined"
srv_position_defined = None

# server 'wall_follower_switch'
name_wall_follower_switch = "/wall_follower_switch"
srv_wall_follower_switch = None

# topic 'move_base/goal' as Publisher
name_move_base = "/move_base/goal"
topic_move_base = None



''' send the goal to move_base
	
	\param target [geometry_msgs/Point] the target to be reached
'''
def send_goal( target ):
	global topic_move_base
	
	# prepare the goal
	msg = MoveBaseActionGoal( )
	msg.goal.target_pose.header.frame_id = 'map'
	msg.goal.target_pose.pose.position.x = target.x
	msg.goal.target_pose.pose.position.y = target.y
	msg.goal.target_pose.pose.orientation.w = 1.0
	
	# send through the topic 
	topic_move_base.publish( msg )



''' require and reach one random position among the available ones. 

'''
def reach_random_pos():
	global srv_get_point
	global srv_check_position
	
	# get the position to reach from the server
	rospy.loginfo( " [user console] asking for a position to '%s' ... ", name_get_point )
	target = ( srv_get_point( get_pointRequest() ) ).position
	rospy.loginfo( " [user console] selected: ( %f, %f )", target.x, target.y )
	
	reach_this( target )



''' reach the given target. 
	
	\param target [geometry_msgs/Point] the target to be reached
'''
def reach_this( target ):
	global srv_check_position
	
	# send the goal to move_base
	rospy.loginfo( " [user console] sending to move_base ... " )
	send_goal( target )
	
	# wait until the target is reached
	rospy.loginfo( " [user console] reaching the target ... " )
	
	reached = False
	while not reached:
		( rospy.Rate( 1 ) ).sleep()
		reached = srv_check_position( check_positionRequest( target, 2 ) ).reached
	
	rospy.loginfo( " [user console] target: REACHED." )



'''reach a position given by the user via console. 

'''
def reach_user_pos():
	global name_position_defined, srv_position_defined
	
	# ask the user for a target
	rospy.loginfo( " [user console] Give me a target:" )
	print( "available: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)]" )
	target = Point()
	target.x = float(input( "\tx : " ))
	target.y = float(input( "\ty : " ))
	
	# check the position
	res = srv_position_defined( target )
	if res.defined :
		rospy.loginfo( " [user console] selected: ( %f, %f )", target.x, target.y )
		reach_this( target )
	else:
		rospy.loginfo( " [user console] ERROR: not a valid position. " )



'''activate/deactivate the wall follower.

'''
def wall_follow():
	global name_wall_follower_switch, srv_wall_follower_switch
	
	# activate the wall_follow service
	srv_wall_follower_switch( True )
	
	# wait
	rospy.loginfo( " [user console] wall_follower active! Press enter to stop." )
	input( "" )
	
	# stop the wall follower
	srv_wall_follower_switch( False )
	rospy.loginfo( " [user console] wall_follower stopped. " )



'''stop the ongoing movement, if any
	
'''
def last_pos():
	pass



'''select a motion planning algorithm.

'''
def change_motion_planning_algorithm():
	pass



'''print a help on the screen.

'''
def print_help():
	global commands, commands_info
	
	print( "\nAvailable commands:" )
	
	for i in range( 1, len( commands ) ):
		print( "\t" + commands[i] + ":" )
		print( "\t\t" + commands_info[i] )



'''Ask to the user the next command. 

'''
def main():
	global commands
	
	print( "\n\tready!\n" )
	
	while not rospy.is_shutdown():
		# wait for one command from the user
		cmd = input( "-> " )
		
		# first, clean the command
		cmd = cmd.strip( )
		
		# then parse it
		cmd_idx = -1
		if cmd.isnumeric( ):
			# take the corresponding integer
			i = int( cmd )
			
			# if the integer is a command, set the command
			if i > 0 and i < len( commands ):
				cmd_idx = i
		else:
			# case-insensitive
			cmd = cmd.lower( )
			
			# interpretation
			if cmd in commands:
				cmd_idx = commands.index( cmd )
		
		# finally, call the proper function
		if cmd_idx < 0:
			rospy.loginfo( " [user console] ERROR: not a valid command!" )
			print( "need infos? write ''help" )
			continue
		elif cmd_idx == 1:
			reach_random_pos( )
		elif cmd_idx == 2:
			reach_user_pos( )
		elif cmd_idx == 3:
			wall_follow( )
		elif cmd_idx == 4:
			last_pos( )
		elif cmd_idx == 5:
			change_motion_planning_algorithm( )
		elif cmd_idx == 6:
			print_help( )
		elif cmd_idx == 7:
			# close the program
			rospy.loginfo( " [user console] exit command received" )
			break



def on_shut_msg():
	rospy.loginfo( " [user console] (shutdown) CLosing ..." )



if __name__ == "__main__":
	# global name_move_base, topic_move_base
	# global name_get_point, srv_get_point
	# global name_check_position, srv_check_position
	
	rospy.init_node( "user_console" )
	rospy.on_shutdown( on_shut_msg )
	
	# require the target 'move_base/goal'
	rospy.loginfo( " [user console] getting topic %s ...", name_move_base )
	topic_move_base = rospy.Publisher( name_move_base, MoveBaseActionGoal, queue_size=1 )
	rospy.loginfo( " [user console] getting topic %s ... OK", name_move_base )
	
	# require the server 'get_point'
	rospy.loginfo( " [user console] getting server %s ...", name_get_point )
	rospy.wait_for_service( name_get_point )
	srv_get_point = rospy.ServiceProxy( name_get_point, get_point )
	rospy.loginfo( " [user console] getting server %s ... OK", name_get_point )
	
	# require the server 'check_position'
	rospy.loginfo( " [user console] getting server %s ...", name_check_position )
	rospy.wait_for_service( name_check_position )
	srv_check_position = rospy.ServiceProxy( name_check_position, check_position )
	rospy.loginfo( " [user console] getting server %s ... OK", name_check_position )
	
	# require the server 'position_defined'
	rospy.loginfo( " [user console] getting server %s ...", name_position_defined )
	rospy.wait_for_service( name_position_defined )
	srv_position_defined = rospy.ServiceProxy( name_position_defined, position_defined )
	rospy.loginfo( " [user console] getting server %s ... OK", name_position_defined )
	
	# require the server 'wall_follower_switch'
	rospy.loginfo( " [user console] getting server %s ...", name_wall_follower_switch )
	rospy.wait_for_service( name_wall_follower_switch )
	srv_wall_follower_switch = rospy.ServiceProxy( name_wall_follower_switch, SetBool )
	rospy.loginfo( " [user console] getting server %s ... OK!", name_wall_follower_switch )
	
	try:
		main()
	except rospy.ROSException:
		rospy.loginfo( " [user console] Raised an Exception ... " )
