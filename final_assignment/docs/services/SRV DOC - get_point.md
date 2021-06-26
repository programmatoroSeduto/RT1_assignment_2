\page DOCSRV_get_point /get_point

\ref services Back to services

# SERVICE - /get_point

**Provided by:**

- `points_manager.py`

**Used by:**

- `reach_random_pos_service.py`
- `reach_user_pos_service.py`
- `bug0.py`

## Semantic

The service returns a planar point (x, y) randomly choosen within a set of pre-defined targets. 

## Service code

### SRV file

```yaml
# servicefile name 'get_point.srv'

#...empty...

---

# the point, randomly choosen
geometry_msgs/Point position
```

### Set-up (Python)

```python
from final_assignment.srv import get_point, get_pointRequest, get_pointResponse

## name of the service 'get_point'
name_get_point = "/get_point"

## === CLIENT SIDE ===

## call-point of the service 'get_point'
srv_get_point = None

# service 'get_point'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_get_point )
rospy.wait_for_service( name_get_point )
srv_get_point = rospy.ServiceProxy( name_get_point, get_point )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_get_point )
```

### How to use the service (Python)

Empty request; non-blocking; it cannot fail. 

```python
global name_get_point, srv_get_point

res = srv_get_point( )
# res.position
```