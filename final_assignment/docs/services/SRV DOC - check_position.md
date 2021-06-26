\page DOCSRV_check_position /check_position

\ref services Back to services

# SERVICE - /check_position

**Provided by:**

- `check_position.py`

**Used by:**

- `reach_random_pos_service.py`
- `reach_user_pos_service.py`

## Semantic

This service provides some useful informations for checking the distance from the target and for understanding the position of the robot in the environment. The service interprets the informations from topic */odom*. 

## Service code

### SRV file

```yaml
# server file name 'check_position'

# check only (ALWAYS SET THIS FIELD!)
bool check_only

# the tolerance, which must be >=0
float32 tol

# the point you want to reach (only with check_only = FALSE)
geometry_msgs/Point target

---

# the actual position of the robot (point)
geometry_msgs/Point actual_position

# is the target reached given the tolerance?
bool reached

# valid operation?
bool success

# real distance from the target
float64 distance
```

### Set-up (Python)

```python
from final_assignment.srv import check_position, check_positionRequest, check_positionResponse

## === CLIENT SIDE ===

## name of the service 'check_position'
name_check_position = "/check_position"

## call-point of the service 'check_position'
srv_check_position = None

# service 'check_position'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_check_position )
rospy.wait_for_service( name_check_position )
srv_check_position = rospy.ServiceProxy( name_check_position, check_position )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_check_position )
```

### How to use the service (Python)

Simple check of the position of the robot. It cannot fail. Non-blocking. 

```python
global name_check_position, srv_check_position

req = check_positionRequest( )
req.check_only = True

res = srv_check_position( req )
```

Check the distance from a given target.

It could fail if the tolerance is negative or zero. Check *res.success* . 

Non-blocking service. 

```python
req = check_positionRequest( )
req.check_only = False
# req.tol = int(tol)
# req.target = Point(x, y, z)

res = srv_check_position( req )
# check the field res.success
```