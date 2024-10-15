## RRM

# Run

```
ros2 run rrm_sim robot_sim_node
```

# Topics

**/joint_states** (sensor_msgs/msg/JointState) - periodically publishing current joint state

**/move_command** (rrm_msgs/msg/Command) - command which move selected joint (value joint_id) to desired position using the message:

````
int32 joint_id 
float64 position   # rad
````
example in command line:
````
ros2 topic pub /move_command rrm_msgs/msg/Command "{joint_id: 1, position: 1.5}" -1
````

# Services 

**/move_command** (rrm_msgs/srv/Command) - command which move all joints to desired positions with defined velocities
using the message:
````
float64[] positions      # rad
float64[] velocities     # rad/s
---
int32 result_code
string message
````
Example in command line:
````
ros2 service call /move_command rrm_msgs/srv/Command "{positions: [1.5,0.7,0.7], velocities: [1.0,1.0,1.0]}"
````
