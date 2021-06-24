#! /usr/bin/env python 



import rospy
from std_srvs.srv import SetBool
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse
from final_assignment.srv import user_target, user_targetRequest, user_targetResponse
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse
from final_assignment.srv import bug0_status, bug0_statusRequest, bug0_statusResponse
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

## name of the service 'bug0_switch'
name_bug0_switch = "/bug0_switch"

## name of the service 'bug0_status'
name_bug0_status = "/bug0_status"

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

## use bug0 or move_base
use_bug0 = False





# --------------------------------- FUNCTIONS

def reach_random_pos( ):
	''' activate the node 'reach_random_pos_service.py'.
	
	'''
	global node_name, use_bug0
	global reach_random_pos_active, robot_busy
	global name_reach_random_pos_switch, srv_reach_random_pos_switch
	global name_bug0_switch, srv_bug0_switch
	
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
			# activate the service
			if not use_bug0:
				srv_reach_random_pos_switch( True, False )
			else:
				srv_bug0_switch( True, False )
			
			# set the robot as busy
			robot_busy = True
			reach_random_pos_active = True
			
			rospy.loginfo( " [%s] reach_random_pos STARTED using '%s'", node_name, ( "bug0" if use_bug0 else "move_base" ) )



def stop_reach_random_pos( ):
	'''
		stop the activity 'reach_random_pos'
	'''
	global use_bug0
	global robot_busy, reach_random_pos_active
	global name_bug0_switch, srv_bug0_switch
	global name_reach_random_pos_switch, srv_reach_random_pos_switch
	
	# send the stop signal to the node
	rospy.loginfo( " [%s] sending the stop signal... ", node_name )
	res = None
	if not use_bug0:
		res = srv_reach_random_pos_switch( False, False )
	else:
		res = srv_bug0_switch( False, False )
	
	# then wait for the robot to reach the final position
	if res.success or use_bug0:
		# immediate end of the activity
		pass
	else:
		# wait until the robot has reached the position
		rospy.loginfo( " [%s] waiting for the robot to end the path... ", node_name )
		while res.in_progress:
			rospy.sleep( rospy.Duration( 0, 500 ) )
			res = srv_reach_random_pos_switch( False, False )
	
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
	global name_bug0_switch, srv_bug0_switch
	global name_bug0_status, srv_bug0_status
	
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
	else:
		rospy.logerr( " [user console] ERROR: not a valid position. " )
		return
	
	# reach the position
	rospy.loginfo( " [%s] reaching the position... ", node_name )
	if not use_bug0:
		srv_user_target( target )
	else:
		reach_user_pos_bug0( target )
	
	rospy.loginfo( " [%s] Arrived. ", node_name )



def reach_user_pos_bug0( target ):
	''' reach a given target using bug0 algorithm
		\param target (geometry_msgs/Point)
	'''
	global node_name
	global name_user_target, srv_user_target
	global name_bug0_switch, srv_bug0_switch
	global name_bug0_status, srv_bug0_status
	
	# first of all, put the target into the parameter server
	rospy.set_param("des_pos_x", target.x)
	rospy.set_param("des_pos_y", target.y)
	
	# activate bug0 in once_only mode
	res = srv_bug0_switch( True, only_once=True )
	
	# wait until the robot has reached the goal
	goal_reached = ( srv_bug0_status( ) ).reached
	while not goal_reached:
		rospy.sleep( rospy.Duration( 0, 500 ) )
		goal_reached = ( srv_bug0_status( ) ).reached



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
	global node_name, use_bug0, robot_busy
	
	# you can't change the motion planning algorithm while you're using it!
	if robot_busy:
		rospy.logwarn( " [%s] ATTENTION: the robot is busy now! ", node_name )
		print( "\tPlease turn off the previous command (use last_pos) before calling this one. " )
		
		return 
	
	rospy.loginfo( "[%s] actually you're using this motion planning algorithm: '%s' ", node_name, ('bug0' if use_bug0 else 'move_base') )
	print( "[%s] would you like to change with this? '%s' [Y/n]" % ( node_name, ('bug0' if not use_bug0 else 'move_base') ) )
	answer = input( "[Y/n]" )
	
	if answer == "Y" or answer == "y":
		# change planning algorithm
		use_bug0 = not use_bug0
	else:
		# don't change algorithm
		pass
	
	rospy.loginfo( "[%s] using algorithm: '%s' ", node_name, ('bug0' if use_bug0 else 'move_base') )



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

## call-point of the service 'bug0_switch'
srv_bug0_switch = None

## call-point of the service 'bug0_status'
srv_bug0_status = None



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
	
	# service 'bug0_switch'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_bug0_switch )
	rospy.wait_for_service( name_bug0_switch )
	srv_bug0_switch = rospy.ServiceProxy( name_bug0_switch, switch_service )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_switch )
	
	# service 'bug0_status'
	rospy.loginfo( " [%s] getting service %s ...", node_name, name_bug0_status )
	rospy.wait_for_service( name_bug0_status )
	srv_bug0_status = rospy.ServiceProxy( name_bug0_status, bug0_status )
	rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_status )
	
	try:
		rospy.loginfo( " [%s] is ONLINE", node_name )
		main()
	except rospy.ROSException:
		# ... properly manage the exception
		# https://docs.python.org/3/tutorial/errors.html
		rospy.logwarn( " [%s] Raised an Exception ... ", node_name )
