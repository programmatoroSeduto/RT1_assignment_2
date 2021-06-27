\page DOCSRV_bug0_switch /bug0_switch

\ref services Back to services

# SERVICE - /bug0_switch

**Provided by:**

- `bug0.py`

**Used by:**

- `user_console.py`

## Semantic

Tihis service is used for activating, or deactivating, the `bug0.py` node. 

When it is turned on, the request is immediately accepted. Note well:

- if the flag `only_once` is `True`, the algorithm takes the target from the parameter server (parameters `des_pos_x` and `des_pos_y`) with no checking on the validity (the node `user_console` is supposed to perform these checkings). 
- otherwise, the robot ciclically asks for a random position, and then tries to reach it.

The turn-off request is always immediately performed: the robot stops in the actual position. 

## Service code

### SRV file

See the message \ref DOCSRV_reach_random_pos_switch "switch_service.srv".

### Set-up (Python)

```python
from final_assignment.srv import switch_service, switch_serviceRequest, switch_serviceResponse

## name of the service 'reach_random_pos_switch'
name_reach_random_pos_switch = "/reach_random_pos_switch"

## === CLIENT SIDE ===

## call-point of the service 'reach_random_pos_switch'
srv_reach_random_pos_switch = None

# service 'reach_random_pos_switch'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_reach_random_pos_switch )
rospy.wait_for_service( name_reach_random_pos_switch )
srv_reach_random_pos_switch = rospy.ServiceProxy( name_reach_random_pos_switch, switch_service )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_reach_random_pos_switch )
```

### How to use the service (Python)

Both the requests cannot fail in relevant cases; field *in_progress* is no use.

Non-blocking call. `res.success = False` when the node is already active. 

```python
global name_bug0_switch, srv_bug0_switch

# Turn on, in only once mode
#    REMEMBER WELL: set a destination before calling the service!
rospy.set_param("des_pos_x", x)
rospy.set_param("des_pos_y", y)
res = srv_bug0_switch( True, only_once=True )
# res.success = False when the node is already active. 

# Turn on, random target mode
res = srv_bug0_switch( True, False )
# res.success = False when the node is already active. 

# Turn off the service
srv_bug0_switch( False, False )
```