# Implementing the safety bubble method

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running. Here as an example for the package of 'reactive control - gap follow', we put the package inside the sim container by mounting the file path in the 'docker-compse.yml' by the line '../reactive_control/gap_follow:/sim_ws/src/gap_follow' under 'volumes' part. Below are the commands to build the agent and make it run within another bash session:

Source both the ROS2 setup script and the local workspace setup script, and then select to build the package. After build succeed, in the same bash, make sure to source both the ROS2 setup script and the local workspace setup script again (might not be the most elegant way), and for this case run the launch file for the node:
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build --packages-select safety_bubble

source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch safety_bubble safety_bubble_launch.py
```
