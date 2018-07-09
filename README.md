# ros-gazebo-step
Exposes a ROS Service that allows for stepping of the Gazebo Simulator in terms of number of physics ODE iterations (which is useful for an RL setup).

Normally, ROS's `spin()` will run at irregular speeds (its hardware / CPU usage dependent), which means (in an RL setup) that you can't guarantee that the same amount of "time" (usually counted in # of iterations) has passed from one timestep to another.

## Example

```python
from std_msgs.msg import Int32

# Gazebo 
step_world = rospy.Publisher('/gazebo_client/step_world', Int32, queue_size=5)
step_message = Int32()
step_message.data = frame_skip

# Queue your actions....

step_world.publish(step_message)

# Note: No more rospy.spin() ! 
```

## Usage
1. Throw the `step_plugin` directory into your `catkin_ws/src`
2. In your Gazebo `world` file, add this line:
`<plugin name='step_plugin' filename='libstep_plugin.so'/>`
3. In your ROSLaunch file, add this line:
`<env name="GAZEBO_PLUGIN_PATH" value="$(find step_plugin)"/>`
4. Build as usual.

