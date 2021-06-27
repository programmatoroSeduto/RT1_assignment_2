\page services Services

\ref index "Back to Home"

# Services

## List of services

Here are the services created along with the project: 

### reach_?_pos + move_base

- \subpage DOCSRV_reach_random_pos_status "/reach_random_pos_status --- status of reach_random_pos_service"
- \subpage DOCSRV_reach_random_pos_switch "/reach_random_pos_switch --- Turn on/off the node reach_random_pos_service.py"
- \subpage DOCSRV_user_target "/user_target --- let the user to reach a target."

### bug0 and wall follower

- \subpage DOCSRV_bug0_switch "/bug0_switch --- turn on or off the bug0 motion planning algorithm"
- \subpage DOCSRV_bug0_status "/bug0_status --- Check the status of the component bug0"
- \subpage DOCSRV_wall_follower_switch "/wall_follower_switch --- turn on or off the wall follower service"

### Position Checking and Points manager

- \subpage DOCSRV_check_position "/check_position --- checkings on the actual position from /odom"
- \subpage DOCSRV_get_point "/get_point --- get a random target from a pre-defined set of targets. "
- \subpage DOCSRV_position_defined "/position_defined --- is this target a predefined one?"


## How to use this documentation

As you can see, the description of each service is made up of many sections:

- **Provided by** the node which provides the service. 
- **Used by** nodes inside the project which use the service.
- **Semantic** an overview about how the server should be used.
- **SRV file** the code of the SRV file, with useful comments
- **set-up** code which could help you in requiring the service. If you want to use these nodes, simply copy and paste the code inside your module following this steps.
	1. put the `include` section among all the other includes
	2. copy and paste the variales in a global namespace
	3. (CLIENT SIDE) put the ServiceProxy code inside your `__main__` section
	4. (SERVER SIDE if available) put the Service code inside your `__main__` section
- **How to use the service** another piece of code you can use for interacting with the service. You don't need to re-write code: just copy and past from here. 

For using the loginfo functions, be sure there is a variable `node_name` inside your node, containing the name of the node, or a different string if you want.