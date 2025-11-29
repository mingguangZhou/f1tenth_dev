# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

## Without an NVIDIA gpu:

**Install the following dependencies:**

If your system does not support nvidia-docker2, noVNC will have to be used to forward the display.
- Again you'll need **Docker**. Follow the instruction from above.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.

**Installing the simulation:**

1. Clone this repo 
2. Bringup the novnc container and the sim container with docker-compose (with '--build' at the first time):
```bash
cd f1tenth_gym_ros
docker compose up --build
``` 
if rebuild for the docker image is needed, turn down the docker-compose before building:
```bash
docker compose down
``` 
3. In a separate terminal, run the following, and you'll have the a bash session in the simulation container. `tmux` is available for convenience.
```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```
4. In your browser, navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html), you should see the noVNC logo with the connect button. Click the connect button to connect to the session.

## With an NVIDIA gpu:

**Install the following dependencies:**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps you won't have to prepend your docker and docker-compose commands with sudo.
- **nvidia-docker2**, follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if you have a support GPU. It is also possible to use Intel integrated graphics to forward the display, see details instructions from the Rocker repo. If you are on windows with an NVIDIA GPU, you'll have to use WSL (Windows Subsystem for Linux). Please refer to the guide [here](https://developer.nvidia.com/cuda/wsl), [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html), and [here](https://dilililabs.com/zh/blog/2021/01/26/deploying-docker-with-gpu-support-on-windows-subsystem-for-linux/).
- **rocker** [https://github.com/osrf/rocker](https://github.com/osrf/rocker). This is a tool developed by OSRF to run Docker images with local support injected. We use it for GUI forwarding. If you're on Windows, WSL should also support this. On ubuntu 22.04 can be simply installed by pip:
```bash
pip install rocker
```
If 'rocker' is installed in '~/.local/bin/', for which has not been added to PATH in all enviroment, run the following commands:
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

**Installing the simulation:**

1. Clone this repo
2. Build the docker image by:
```bash
cd f1tenth_gym_ros
docker build -t f1tenth_gym_ros -f Dockerfile .
```
3. To run the containerized environment, start a docker container by running the following. (example showned here with nvidia-docker support). By running this, the current directory that you're in (should be `f1tenth_gym_ros`) is mounted in the container at `/sim_ws/src/f1tenth_gym_ros`. Which means that the changes you make in the repo on the host system will also reflect in the container.
```bash
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```
For the current setup the 'f1tenth_rocker_start.sh' script shall be used directly to start the container with other packages mounted on:
```bash
./f1tenth_rocker_start.sh
```
In a separate terminal, run the following, and you'll have the a bash session in the simulation container.
```bash
docker exec -it f1tenth_gym_ros_rocker /bin/bash
```

# Launching the Simulation

1. `tmux` is included in the contianer, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

# Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running. For 'without an NVIDIA GPU', as an example for the package of 'reactive control - gap follow', we put the package inside the sim container by mounting the file path in the 'docker-compse.yml' by the line '../reactive_control/gap_follow:/sim_ws/src/gap_follow' under 'volumes' part. For 'without an NVIDIA GPU', the packages are mounted in the similar way in the 'f1tenth_rocker_start.sh' scripts. Below are the commands to build the agent and make it run within another bash session in both situations:

1. Source both the ROS2 setup script and the local workspace setup script, and then select to build the package(s):
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon build --packages-select boundary_detection path_following
```
2. After build succeed, in the same bash, make sure to source the install setup again (might not be the most elegant way), and for this case run the launch file for the node:
```bash
source install/local_setup.bash
ros2 launch path_following path_following_launch.py 
```
The docker container should also had installed the GDB, so you can also start a GDB session rather than launch file (to source again seems not needed for this way):
```bash
gdb --args /sim_ws/install/gap_follow/lib/gap_follow/reactive_node --ros-args -r __node:=reactive_node --params-file /sim_ws/install/gap_follow/share/gap_follow/config/params.yaml
```

- The second one is to create a new ROS 2 container for you agent node. Then create your own package and nodes inside. Launch the sim container and the agent container both. With default networking configurations for `docker`, the behavior is to put The two containers on the same network, and they should be able to discover and talk to each other on different topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node. You'll also have to put your container on the same network as the sim and novnc containers.
