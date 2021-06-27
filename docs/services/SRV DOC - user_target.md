\page DOCSRV_user_target /user_target

\ref services Back to services

# SERVICE - /user_target

**Provided by:**

- `reach_user_pos_service.py`

**Used by:**

- `user_console.py`

## Semantic

This service lets the program to reach, using move_base, a point of the environment. It is used for letting the user to manually give a target to reach to teh robot. 

The server itself performs no checkings on the given target: it must be the node `user_console.py` the one which checks if the position is defined or not. 

\see check_position

## Service code

### SRV file

```yaml
# service file name 'user_target'

# target
geometry_msgs/Point target

---

# success?
bool success
```

### Set-up (Python)

```python
from final_assignment.srv import user_target, user_targetRequest, user_targetResponse

# name of the service 'user_target'
name_user_target = "/user_target"

## === CLIENT SIDE ===

# entry point of the service 'user_target'
srv_user_target = None

# require the server 'user_target'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_user_target )
rospy.wait_for_service( name_user_target )
srv_user_target = rospy.ServiceProxy( name_user_target, user_target )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_user_target )

## === SERVER SIDE ===

# SERVER
# require the server 'user_target'
rospy.loginfo( " [%s] advertising service %s ...", node_name, name_user_target )
rospy.Service( name_user_target, user_target, srv_user_target )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_user_target )
```

### How to use the service (Python)

The request always succeed.

**BLOCKING REQUEST**: the caller has to wait until the robot has reached the target position. 

```python
global name_user_target, srv_user_target

req = user_targetRequest(  )
# req.target = Point(x,y,z)

# BLOCKING SERVICE!
# wait until the robot has reached the target
res = srv_user_target( req )

```