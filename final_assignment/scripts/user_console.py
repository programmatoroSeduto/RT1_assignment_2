#! /usr/bin/env python 

import rospy
from std_srvs.srv import SetBool
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse


# --------------------------------- DATA

## name of this node
node_name = "user_console"

## name of the server 'wall_follower_switch'
name_wall_follower_switch = "/wall_follower_switch"

## name of the service 'reach_random_pos_switch'
name_reach_random_pos_switch = "/reach_random_pos_switch"

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

## last command index
last_command_idx = -1

## is there any movement ongoing?
robot_busy = False

## is reach_random_pos active?
reach_random_pos_active = False





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
		rospy.logwarn( " [%] ATTENTION: each_random_pos already active. ", node_name )
	else:
		 if robot_busy:
			# you cannot call this command when the robot is busy
			# before calling this, please turn off the previous operation
			rospy.logwarn( " [%] ATTENTION: another algorithm is running actually. ", node_name )
			print( "\nPlease turn off the previous one before calling this command. " )
			pass
		else:
			# activate the service 'reach_random_pos_service'
			srv_reach_random_pos_switch( True )
			
			# set the robot as busy
			robot_busy = True
			reach_random_pos_active = True
			
			rospy.loginfo( " [%] reach_random_pos is ON. ", node_name )



def reach_user_pos():
	''' ask to the user a position to reach, then reach it
		
	'''
	global node_name
	pass



def wall_follow():
	'''activate/deactivate the wall follower.
		
	'''
	global node_name
	pass



def wall_follow():
	'''activate/deactivate the wall follower.

	'''
	global node_name
	global name_wall_follower_switch, srv_wall_follower_switch
	
	# activate the wall_follow service
	srv_wall_follower_switch( True )
	
	# wait
	rospy.loginfo( " [user console] wall_follower active! Press enter to stop." )
	input( "" )
	
	# stop the wall follower
	srv_wall_follower_switch( False )
	rospy.loginfo( " [user console] wall_follower stopped. " )



def last_pos():
	'''stop the ongoing movement, if any
		
	'''
	global node_name
	pass



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
		print( "\t [" + i + "] " + commands[i] + ":" )
		print( "\t\t" + commands_info[i] )



# --------------------------------- SERVICES

# call point of the server 'wall_follower_switch'
srv_wall_follower_switch = None

## call-point of the service 'reach_random_pos_switch'
srv_reach_random_pos_switch = None



# ... callbacks ...



# --------------------------------- TOPICS

## ...



# --------------------------------- WORKING CYCLE

def main():
	'''Ask to the user the next command. 

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
	rospy.loginfo( " [%s] closing...", node_name )



if __name__ == "__main__":
	rospy.init_node( node_name )
	rospy.on_shutdown( cbk_on_shutdown )
	
	# require the server 'wall_follower_switch'
	rospy.loginfo( " [%] getting service %s ...", node_name, name_wall_follower_switch )
	rospy.wait_for_service( name_wall_follower_switch )
	srv_wall_follower_switch = rospy.ServiceProxy( name_wall_follower_switch, SetBool )
	rospy.loginfo( " [%] service %s ... OK", node_name, name_wall_follower_switch )
	
	# service 'reach_random_pos_switch'
	rospy.loginfo( " [%] getting service %s ...", node_name, name_reach_random_pos_switch )
	rospy.wait_for_service( name_reach_random_pos_switch )
	srv_reach_random_pos_switch = rospy.ServiceProxy( name_reach_random_pos_switch, switch_service )
	rospy.loginfo( " [%] service %s ... OK", node_name, name_reach_random_pos_switch )
	
	try:
		main()
	except rospy.ROSException:
		# ... properly manage the exception
		# https://docs.python.org/3/tutorial/errors.html
		rospy.loginfo( " [user console] Raised an Exception ... " )
