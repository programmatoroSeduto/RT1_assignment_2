\page howto-commands Command Line Interfaces - Commands 

# Command Line Interface of the project - an Overview

## Available Functionalities

Here are the implemented functionalities you can use. The command line interface is case-insensitive, and accepts both literal command and number command. 

1. `reach_random_pos` : the controller asks for a new target position to a dedicated server, and drives the robot towards the position using one of the available motion planning algorithm *bug0* and *move_base*. The command starts a backgrond process. 
2. `reach_user_pos` : foreground process. The program asks the use for a target, then sends it to the motion planning algorithm and drives the robot. 
3. `wall_follow` : the robot starts to follow the external walls of the environment. Background process.
4. `last_pos` : the command stops any background process. 
5. `change_motion_planning_algorithm` : you can decide the motion planning algorith between *bug0* or *move_base*.
6. `help` : print an help on the screen. 
7. `exit` : close the command line interface. 

## Base Commands

When this symbol appears on the screen, you can start typing the commands:

```

->	...command here

```

Commands are always case-insensitive. Moreover, you can write either the command or the number corresponding to the command. I prefer numbers, because I'm terribly lazy ;-)

Two basic commands. **exit** for closing the application:

```

-> exit
[INFO] [1624723424.332583, 121.329000]:  [user console] exit command received
[INFO] [1624723424.335212, 121.331000]:  [user_console] is OFFLINE

```

Also the corresponding number works. For `exit`, the number is 7:

```

-> 6
[INFO] [1624723424.332583, 121.329000]:  [user console] exit command received
[INFO] [1624723424.335212, 121.331000]:  [user_console] is OFFLINE

```

Another useful command is `help` (number 6). This is the corresponding output:

```

-> 6

Available commands:
	 [no.1] reach_random_pos:
		periodically ask a position to the server, then try and reach it.
	 [no.2] reach_user_pos:
		ask to the user a position, then try and reach it. 
	 [no.3] wall_follow:
		use the component wall_follower.
	 [no.4] last_pos:
		Stop any ongoing movement
	 [no.5] change_motion_planning_algorithm:
		select your motion planning algorithm between 'move_base' and 'bug0'
	 [no.6] help:
		print this help
	 [no.7] exit:
		Close this program.
-> 

```

## Number 5 : change_motion_planning_algorithm

The default algorithm is *move_base*, but you can change it simply invoking this command. Let's change from *move_base* to *bug0*:

```

-> 5
[INFO] [1624723732.513973]: [user_console] actually you're using this motion planning algorithm: 'move_base' 
[user_console] would you like to change with this? 'bug0' [Y/n]
[Y/n]y
[INFO] [1624723735.415190]: [user_console] using algorithm: 'bug0' 
-> 

```

In the same way, you can change from *bug0* to *move_base*:

```

-> 5      
[INFO] [1624723791.762845]: [user_console] actually you're using this motion planning algorithm: 'bug0' 
[user_console] would you like to change with this? 'move_base' [Y/n]
[Y/n]y
[INFO] [1624723793.197886]: [user_console] using algorithm: 'move_base' 
-> 

```

A very important remark: **you cannot change the planning angorithm when a background process is running**. The output will be this:

```

-> 1      
[INFO] [1624723822.944849]:  [user_console] reach_random_pos STARTED using 'move_base'
-> 5
[WARN] [1624723825.508154]:  [user_console] ATTENTION: the robot is busy now! 
	Please turn off the previous command (use last_pos) before calling this one. 
->  

```

