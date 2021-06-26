\page DOCSRV_reach_random_pos_status /reach_random_pos_status

\ref services Back to services

# SERVICE - /reach_random_pos_status

**Provided by:**

- `reach_random_pos_service.py`

**Used by:**

- `user_console.py`

## Semantic

This service is used for asking the internal status of the node `reach_random_pos_service.py`. From there, you can obtain many useful information. See the message for more informations. 

\see DOCSRV_reach_random_pos_switch meaning of *last_pos_signal*.

## Service code

### SRV file

```yaml
# service 'reach_random_pos_status'

# Empty request

---

# is the node active?
bool is_active

# the actual target
geometry_msgs/Point target

# the distance from the point to reach
float64 distance

# the actual position (point)
geometry_msgs/Point actual_position

# is it the last position? (true/false)
bool last_pos_signal
```

### Set-up (Python)

```python
from final_assignment.srv import reach_random_pos_status, reach_random_pos_statusRequest, reach_random_pos_statusResponse

## name of the service 'reach_random_pos_status'
name_reach_random_pos_status = "/reach_random_pos_status"

## === CLIENT SIDE ===

## call-point of the service 'reach_random_pos_status'
srv_reach_random_pos_status = None

# require the server 'check_position'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_reach_random_pos_status )
rospy.wait_for_service( name_reach_random_pos_status )
srv_reach_random_pos_status = rospy.ServiceProxy( name_reach_random_pos_status, reach_random_pos_status )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_status )

## === SERVER SIDE ===

# require the server 'check_position'
rospy.loginfo( " [%s] advertising service %s ...", node_name, name_reach_random_pos_status )
rospy.Service( name_reach_random_pos_status, reach_random_pos_status, srv_reach_random_pos_status )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_status )
```

### How to use the service (Python)

Simply call it using an empty request. 

Non-blocking request. It always succeed. 

```python

global name_reach_random_pos_status, srv_reach_random_pos_status

status = srv_reach_random_pos_status( )

```