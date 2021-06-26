\page DOCSRV_reach_random_pos_switch /reach_random_pos_switch

\ref services Back to services

# SERVICE - /reach_random_pos_switch

**Provided by:**

- `reach_random_pos_service.py`

**Used by:**

- `user_console.py`

## Semantic

This service is used to turn on or off the node `reach_random_pos_service.py`. 

When the service is on, it ciclically asks for a random target, then reach it. 

When a message for turning off the node is issued, the request isn't executed immediately, but is marked as *in progress*: before stopping, the robot reaches the goal. When the goal is reached, the stop signal is executed, and the node is turned off. 

If the server is already on, any last_pos_signal is clean. See How to use this service. 

## Service code

### SRV file

```yaml
# service file name 'switch_service.srv'

# on or off?
bool val

# helpful for bug0
bool only_once

---

# the request was successful
bool success

# this kind of server can return immediately
#    thit is the reason why there is a variable for checkig the status of the request
bool in_progress
```

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

Of course, you can call the service instantiating the request before. 

This server is implemented as a kind of *action*: it doesn't block. 

The activation request always succeed. The other one returns *not success* if the request is in progress. 

```python
global name_reach_random_pos_switch, srv_reach_random_pos_switch

# Turn on the service
srv_reach_random_pos_switch( True, False )
# it always succeed
# if the server is already on, any last_pos_signal is clean
#    and the server return (success, not in progress)

# Turn off the service
res = srv_reach_random_pos_switch( False, False )
# please check res.in_progress
#    if res.in_progress==True check periodically the state
#    until res.success==True
```