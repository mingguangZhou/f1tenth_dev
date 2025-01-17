# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

## Without an NVIDIA gpu:

**Install the following dependencies:**

If your system does not support nvidia-docker2, noVNC will have to be used to forward the display.
- Again you'll need **Docker**. Follow the instruction from above.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.

**Installing the simulation:**

1. Clone this repo 
2. Bringup the novnc container and the sim container with docker-compose:
```bash
$ cd f1tenth_gym_ros
$ docker-compose up --build
``` 
3. In a separate terminal, run the following, and you'll have the a bash session in the simulation container. `tmux` is available for convenience.
```bash
$ docker exec -it f1tenth_gym_ros_sim_1 /bin/bash
```
4. In your browser, navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html), you should see the noVNC logo with the connect button. Click the connect button to connect to the session.

# Launching the Simulation

1. `tmux` is included in the contianer, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:
```bash
/sim_ws#: source /opt/ros/foxy/setup.bash
/sim_ws#: source install/local_setup.bash
/sim_ws#: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

You can then run another node by creating another bash session in `tmux`.

# Configuring the simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave uncahnged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file in the container. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name. See the note below about mounting a volume to see where to put your map file.
- The `num_agent` parameter can be changed to either 1 or 2 for single or two agent racing.
- The ego and opponent starting pose can also be changed via parameters, these are in the global map coordinate frame.

The entire directory of the repo is mounted to a workspace `/sim_ws/src` as a package. All changes made in the repo on the host system will also reflect in the container. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

# Topics published by the simulation

In **single** agent:

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

In **two** agents:

In addition to the topics available in the single agent scenario, these topics are also available:

`/opp_scan`: The opponent agent's laser scan

`/ego_racecar/opp_odom`: The opponent agent's odometry for the ego agent's planner

`/opp_racecar/odom`: The opponent agents' odometry

`/opp_racecar/opp_odom`: The ego agent's odometry for the opponent agent's planner

# Topics subscribed by the simulation

In **single** agent:

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

`/initalpose`: This is the topic for resetting the ego's pose via RViz's 2D Pose Estimate tool. Do **NOT** publish directly to this topic unless you know what you're doing.

TODO: kb teleop topics

In **two** agents:

In addition to all topics in the single agent scenario, these topics are also available:

`/opp_drive`: The opponent agent's drive command via `AckermannDriveStamped` messages. Note that you'll need to publish to **both** the ego's drive topic and the opponent's drive topic for the cars to move when using 2 agents.

`/goal_pose`: This is the topic for resetting the opponent agent's pose via RViz's 2D Goal Pose tool. Do **NOT** publish directly to this topic unless you know what you're doing.

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, run:
```bash
/sim_ws#: ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

# Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running. Here as an example for the package of 'reactive control - gap follow', we put the package inside the sim container by mounting the file path in the 'docker-compse.yml' by the line '../reactive_control/gap_follow:/sim_ws/src/gap_follow' under 'volumes' part. Below are the commands to build the agent and make it run within another bash session:

1. Source both the ROS2 setup script and the local workspace setup script, and then select to build the package:
```bash
/sim_ws#: source /opt/ros/foxy/setup.bash
/sim_ws#: source install/local_setup.bash
/sim_ws#: colcon build --packages-select gap_follow
```
2. After build succeed, in the same bash, make sure to source both the ROS2 setup script and the local workspace setup script again (might not be the most elegant way), and for this case run the launch file for the node:
```bash
/sim_ws#: source /opt/ros/foxy/setup.bash
/sim_ws#: source install/local_setup.bash
/sim_ws#: ros2 launch gap_follow gap_follow_launch.py
```
The docker container should also had installed the GDB, so you can also start a GDB session rather than launch file (to source again seems not needed for this way):
```bash
/sim_ws#: gdb --args /sim_ws/install/gap_follow/lib/gap_follow/reactive_node --ros-args -r __node:=reactive_node --params-file /sim_ws/install/gap_follow/share/gap_follow/config/params.yaml
```

- The second one is to create a new ROS 2 container for you agent node. Then create your own package and nodes inside. Launch the sim container and the agent container both. With default networking configurations for `docker`, the behavior is to put The two containers on the same network, and they should be able to discover and talk to each other on different topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node. You'll also have to put your container on the same network as the sim and novnc containers.
