\page DOCSRV_bug0_status /bug0_status

\ref services Back to services

# SERVICE - /bug0_status

**Provided by:**

- `user_console.py`

**Used by:**

- `execname`

## Semantic

This service lets the program know some useful informations about the progress of the robot towards the target, and some other stuff. See the SRV file for more details. 

\see bug0.py 'Status' is the status number of the state machine bug0: 

- 0=searching a wall, 
- 1=following the wall, 
- 2=target reached. 

## Service code

### SRV file

```yaml
# service file name 'bug0_status.srv'

# Empty request

---

# target
geometry_msgs/Point target_position

# actual position
geometry_msgs/Point actual_position

# actual yaw
float32 actual_yaw

# actual status
int32 status

# active or not
bool active

# distance from the target
float32 distance

# target position reached
bool reached
```

### Set-up (Python)

```python
from final_assignment.srv import bug0_status, bug0_statusRequest, bug0_statusResponse

## name of the service 'bug0_status'
name_bug0_status = "/bug0_status"

## === CLIENT SIDE ===

## call-point of the service 'bug0_status'
srv_bug0_status = None

# service 'bug0_status'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_bug0_status )
rospy.wait_for_service( name_bug0_status )
srv_bug0_status = rospy.ServiceProxy( name_bug0_status, bug0_status )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_status )

## === SERVER SIDE ===

# service 'bug0_status'
rospy.loginfo( " [%s] advertising service %s ...", node_name, name_bug0_status )
rospy.Service( name_bug0_status, bug0_status, srv_bug0_status )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_bug0_status )
```

### How to use the service (Python)

Simply call the service, and inspect the returned message. 

Non-blocking call. It cannot fail. 

```python
global name_bug0_status, srv_bug0_status

res = srv_bug0_status( )
```