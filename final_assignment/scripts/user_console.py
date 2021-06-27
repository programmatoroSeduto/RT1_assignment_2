#! /usr/bin/env python 

##
#	@file user_console.py
#	@author Francesco Ganci (S4143910)
#	@brief A user interface for sending commands to a mobile robot.
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#   Here a little command line interface is implemented in order to drive a mobile robot. It can interact with two planning algorithms: bug0 and <b><a href="">move_base</a></b>. <br>
#	see \ref howto-commands "this walkthrough" for understanding what this console can do, and how. 
#	
#	@copyright Copyright (c) 2021
#

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

## 
#	@brief Implementation of the command <b>reach_random_pos</b> [no.1]
#	
#	\details
#	    This function selects the node to activate between move_base and bug0.
#		Then, it sends the proper switch message to the selected motion planning algorithm.
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#   
def reach_random_pos( ):
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



## 
#	@brief Stop the background process `reach_random_pos`
#	
#	\details
#		If <i>move_base</i> is active, the function waits until the target is reached and then turns off the node <i>reach_random_pos_service.py</i>.
#		Otherwise, it immediately stops bug0. 
#
def stop_reach_random_pos( ):
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



## 
#	@brief Implementation of the command <b>reach_user_pos</b> [no.2]
#	
#	\details
#	    This function asks for a target to the user, then checks it. <br>
#		If the position is contained in the set of predefined ones, the command is valid, and a message is issued to the selected motion planning algorithm. <br>
#       After sent the message, the function waits until the target is reached. 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#   
def reach_user_pos():
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



## 
#	@brief Send manually a goal to bug0
#	
#	\param target (geometry_msgs/Point) the target to reach.
#	
#	\details
#		The service bug0 needs a further step: the desired target must be passed via parameter server. <br>
#       So, the function writes on the parameter server, then waits until the goal is reached. 
#   
#   <br>
#   
#   <b>See also:</b> \ref DOCSRV_bug0_status "service bug0 status", in particular how to use it. 
#
def reach_user_pos_bug0( target ):
	global node_name
	global name_user_target, srv_user_target
	global name_bug0_switch, srv_bug0_switch
	global name_bug0_status, srv_bug0_status
	
	# first of all, put the target into the parameter server
	rospy.set_param("des_pos_x", target.x)
	rospy.set_param("des_pos_y", target.y)
	
	# activate bug0 in once_only mode
	res = srv_bug0_switch( True, True )
	
	# wait until the robot has reached the goal
	goal_reached = ( srv_bug0_status( ) ).reached
	while not goal_reached:
		rospy.sleep( rospy.Duration( 0, 500 ) )
		goal_reached = ( srv_bug0_status( ) ).reached



## 
#	@brief Implementation of the command <b>wall_follow</b> [no.3]
#	
#	\param val (bool; default: True) the state of the wall_follow service
#	
#	\details
#	    The function simply activates the wall follow service sending to it a switch message. <br>
#       The same function can turn off the service, passing <i>False</i> as argument. 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#   
def wall_follow( val=True ):
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



## 
#	@brief Implementation of the command <b>last_pos</b> [no.4]
#	
#	\details
#	    Interrupts any ongoing background task whih makes the robot busy. 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#   
def last_pos():
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



## 
#	@brief Implementation of the command <b>change_motion_planning_algorithm</b> [no.5]
#	
#	\details
#	    select a motion planning algorithm between bug0 and move_base. 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#  
def change_motion_planning_algorithm():
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



## 
#	@brief Implementation of the command <b>help</b> [no.6]
#	
#	\details
#	    The function prints an help on the screen. 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#  
def print_help():
	global node_name
	global commands, commands_info
	
	print( "\nAvailable commands:" )
	
	for i in range( 1, len( commands ) ):
		print( "\t [no." + str(i) + "] " + commands[i] + ":" )
		print( "\t\t" + commands_info[i] )



# --------------------------------- SERVICES

## call point of the server 'wall_follower_switch'
srv_wall_follower_switch = None

## call-point of the service 'reach_random_pos_switch'
srv_reach_random_pos_switch = None

## entry point of the service 'user_target'
srv_user_target = None

## call-point of the service 'position_defined'
srv_position_defined = None

## call-point of the service 'bug0_switch'
srv_bug0_switch = None

## call-point of the service 'bug0_status'
srv_bug0_status = None



# --------------------------------- WORKING CYCLE

## 
#	@brief Ask the next command to the user. 
#	
#	\details
#		This function implements a simple while-switch pattern for receiving command from the command line. <br>
#       Commands are all case-insensitive. Also corresponding numbers are allowed (lazy mode). 
#   
#   <br>
#   
#   <b>See also:</b> \ref howto-commands "Command Line Interfaces - Commands"
#  
def main():
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

## 
#	@brief Called when the shutdown signal is raised. 
#	
def cbk_on_shutdown():
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
		rospy.logwarn( " [%s] Raised an Exception ... ", node_name )
