\page DOCSRV_wall_follower_switch /wall_follower_switch

\ref services Back to services

# SERVICE - /wall_follower_switch

**Provided by:**

- `wall_follow_service_m.py`

**Used by:**

- `user_console.py`
- `bug0.py`

## Semantic

This service turns on the all-follow behaviour of the robot.

When the service is on, the node `wall_follow_service_m.py` makes the robot follow the external walls of the area. It directly published on topic */cmd_vel*. 

When the service is turned off, the robot stops immediately in the actual position. 

## Service code

### SRV file

The service uses the standard one [SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html). 

### Set-up (Python)

```python
from std_srvs.srv import SetBool

## name of the server 'wall_follower_switch'
name_wall_follower_switch = "/wall_follower_switch"

## === CLIENT SIDE ===

# call point of the server 'wall_follower_switch'
srv_wall_follower_switch = None

# require the server 'wall_follower_switch'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_wall_follower_switch )
rospy.wait_for_service( name_wall_follower_switch )
srv_wall_follower_switch = rospy.ServiceProxy( name_wall_follower_switch, SetBool )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_wall_follower_switch )
```

### How to use the service (Python)

Non-blocking service. Both the requests cannot fail. 

```python
global name_wall_follower_switch, srv_wall_follower_switch

# accendere il componente
srv_wall_follower_switch( True )

# disattivare il componente
srv_wall_follower_switch( False )
```