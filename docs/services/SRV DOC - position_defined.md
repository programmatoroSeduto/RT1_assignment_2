\page DOCSRV_position_defined /position_defined

\ref services Back to services

# SERVICE - /position_defined

**Provided by:**

- `points_manager.py`

**Used by:**

- `user_console`

## Semantic

The user can select only a position among the ones pre-defined in the node `points_manager.py`.

The server simply checks if the given point is contained or not in that set of targers. 

\see points_manager.py which are the allowed positions

## Service code

### SRV file

```yaml
# service file name 'position_defined.srv'

# the position you want to check
geometry_msgs/Point position

---

# is it in the set?
bool defined
```

### Set-up (Python)

```python
from final_assignment.srv import position_defined, position_definedRequest, position_definedResponse

## === CLIENT SIDE

## name of the service 'position_defined'
name_position_defined = "/position_defined"

## call-point of the service 'position_defined'
srv_position_defined = None

# service 'position_defined'
rospy.loginfo( " [%s] getting service %s ...", node_name, name_position_defined )
rospy.wait_for_service( name_position_defined )
srv_position_defined = rospy.ServiceProxy( name_position_defined, position_defined )
rospy.loginfo( " [%s] service %s ... OK", node_name, name_position_defined )
```

### How to use the service (Python)

Non blocking request. It cannot fail. 

```python
req = position_definedRequest( )
# req.position = Point(x, y, z)

res = srv_position_defined( req )
# res.defined
```