#! /usr/bin/env python 

import rospy
from std_srvs.srv import SetBool
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import user_target, user_targetRequest, user_targetResponse
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse
from geometry_msgs.msg import Point


# --------------------------------- DATA

## name of this node
node_name = "user_console"

## name of the server 'wall_follower_switch'
name_wall_follower_switch = "/wall_follower_switch"

## name of the service 'reach_random_pos_switch'
name_reach_random_pos_switch = "/reach_random_pos_switch"

## name of the service 'user_target'
name_user_target = "/user_target"

## name of the service 'position_defined'
name_position_defined = "/position_defined"

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
  "select your motion planning algorithm between 'move_base' and 'bug0'", \
  "print this help", \
  "Close this program." ]

## last command index
last_command_idx = -1

## is there any movement ongoing?
robot_busy = False

## is reach_random_pos active?
reach_random_pos_active = False

## is wall follow active?
wall_follow_active = False





# --------------------------------- FUNCTIONS

def reach_random_pos():
	''' activate the node 'reach_random_pos_service.py'.
		\TODO integration with bug0
	'''
	global node_name
	global reach_random_pos_active, robot_busy
	global name_reach_random_pos_switch, srv_reach_random_pos_switch
	
	if reach_random_pos_active:
		# the service is already active
		rospy.logwarn( " [%s] ATTENTION: each_random_pos already active. ", node_name )
	else:
		if robot_busy:
			# you cannot call this command when the robot is busy
			# before calling this, please turn off the previous operation
			rospy.logwarn( " [%s] ATTENTION: the robot is busy! ", node_name )
			print( "\tPlease turn off the previous command (use last_pos) before calling this one. " )
		else:
			# activate the service 'reach_random_pos_service'
			srv_reach_random_pos_switch( True )
			
			# set the robot as busy
			robot_busy = True
			reach_random_pos_active = True
			
			rospy.loginfo( " [%s] reach_random_pos STARTED. ", node_name )



def stop_reach_random_pos( ):
	'''
		stop the activity 'reach_random_pos'
	'''
	global robot_busy, reach_random_pos_active
	
	# send the stop signal to the node
	rospy.loginfo( " [%s] sending the stop signal... ", node_name )
	res = srv_reach_random_pos_switch( False )
	
	# then wait for the robot to reach the final position
	if res.success:
		# immediate end of the activity
		pass
	else:
		# wait until the robot has reached the position
		rospy.loginfo( " [%s] waiting for the robot to end the path... ", node_name )
		while res.in_progress:
			rospy.sleep( rospy.Duration( 0, 500 ) )
			res = srv_reach_random_pos_switch( False )
	
	robot_busy = False
	reach_random_pos_active = False
	
	rospy.loginfo( " [%s] reach_random_pos STOPPED. ", node_name )



def reach_user_pos():
	''' ask to the user a position to reach, then reach it
		
	'''
	global node_name
	global name_position_defined, srv_position_defined
	global name_user_target, srv_user_target
	global robot_busy
	
	if robot_busy:
		rospy.logwarn( " [%s] ATTENTION: the robot is busy now! ", node_name )
		print( "\tPlease turn off the previous command (use last_pos) before calling this one. " )
		return
	
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
		# reach_this( target )
		srv_user_target( target )
	else:
		rospy.logerr( " [user console] ERROR: not a valid position. " )



def wall_follow( val=True ):
	'''activate/deactivate the wall follower.
		\param val (bool; default: True) the state of the wall follow service
	'''
	global node_name
	global name_wall_follower_switch, srv_wall_follower_switch
	global robot_busy, wall_follow_active
	
	if val and robot_busy:
		if wall_follow_active:
			rospy.loginfo( " [%s] wall_follow already active. ", node_name )
		else:
			rospy.logwarn( " [%s] ATTENTION: the robot is busy now! ", node_name )
			print( "\tPlease turn off the previous command (use last_pos) before calling this one. " )
		
		return
	
	srv_wall_follower_switch( val )
	
	wall_follow_active = val
	robot_busy = val
	
	rospy.loginfo( " [%s] wall follow is %s ", node_name, ( "ON" if val else "OFF" ) )



def last_pos():
	'''stop the ongoing movement, if any
		
	'''
	global node_name
	global robot_busy
	global reach_random_pos_active, wall_follow_active
	
	if robot_busy:
		if reach_random_pos_active:
			stop_reach_random_pos( )
			
		elif wall_follow_active:
			wall_follow( False )
			
	else:
		# nothing to stop
		rospy.logwarn( " [%s] WARNING: no activity to stop. ", node_name )



def change_motion_planning_algorithm():
	'''select a motion planning algorithm.

	'''
	global node_name
	pass



def print_help():
	'''print a help on the screen.

	'''
	global node_name
	global commands, commands_info
	
	print( "\nAvailable commands:" )
	
	for i in range( 1, len( commands ) ):
		print( "\t [no." + str(i) + "] " + commands[i] + ":" )
		print( "\t\t" + commands_info[i] )



# --------------------------------- SERVICES

# call point of the server 'wall_follower_switch'
srv_wall_follower_switch = None

## call-point of the service 'reach_random_pos_switch'
srv_reach_random_pos_switch = None

# entry point of the service 'user_target'
srv_user_target = None

## call-point of the service 'position_defined'
srv_position_defined = None



# ... callbacks ...



# --------------------------------- TOPICS

## ...



# --------------------------------- WORKING CYCLE

def main():
	'''Ask the next command to the user

	'''
	global node_name
	global commands, last_command_idx
	
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
		
		# store the previous command
		if cmd_idx > 0:
			last_command_idx = cmd_idx
		
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



# --------------------------------- NODE

def cbk_on_shutdown():
	'''
	This is called on the shutdown of the node. 
	'''
	global node_name
	rospy.loginfo( " [%s] is OFFLINE", node_name )



if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutdown )
	
	# require the server 'wall_follower_switch'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_wall_follower_switch )
	rospy.wait_for_service( name_wall_follower_switch )
	srv_wall_follower_switch = rospy.ServiceProxy( name_wall_follower_switch, SetBool )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_wall_follower_switch )
	
	# service 'reach_random_pos_switch'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_reach_random_pos_switch )
	rospy.wait_for_service( name_reach_random_pos_switch )
	srv_reach_random_pos_switch = rospy.ServiceProxy( name_reach_random_pos_switch, switch_service )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_switch )
	
	# require the server 'user_target'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_user_target )
	rospy.wait_for_service( name_user_target )
	srv_user_target = rospy.ServiceProxy( name_user_target, user_target )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_user_target )
	
	# service 'position_defined'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_position_defined )
	rospy.wait_for_service( name_position_defined )
	srv_position_defined = rospy.ServiceProxy( name_position_defined, position_defined )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_position_defined )
	
	try:
		rospy.loginfo( " [%s] is ONLINE", node_name )
		main()
	except rospy.ROSException:
		# ... properly manage the exception
		# https://docs.python.org/3/tutorial/errors.html
		rospy.logwarn( " [%s] Raised an Exception ... ", node_name )
