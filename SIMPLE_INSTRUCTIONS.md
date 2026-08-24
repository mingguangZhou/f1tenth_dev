# SIMPLE_INSTRUCTIONS.md

Last Edit: 2026.08.17

These are the simple daily-use instructions for the current RoboRacer/F1TENTH development setup.

## Recommended atomic workflow

Docker lifecycle is managed from the host with `dk.sh`. ROS packages and
launches are managed inside the container with `f1`. NVIDIA GPU rendering with
a host X11 RViz window is the default.

Build the Docker image only when Docker dependencies change:

```bash
./dk.sh image
```

Start Docker and enter the ROS container:

```bash
./dk.sh start
```

Startup does not install ROS packages and does not require network access.
Normal `f1 build` commands only compile packages. After intentionally adding a
new system dependency to `package.xml`, run `./dk.sh deps` once from the host.

For CPU software rendering and browser/noVNC instead:

```bash
./dk.sh start --cpu
```

Build each ROS group independently inside the container:

```bash
f1 build sim
f1 build auto
```

Run the simulator and RViz in the first container terminal:

```bash
f1 sim
```

The default is the IFAC Roboracer fixture with three fixed obstacles and one
moving traffic car. Remove the moving car while keeping the obstacle map with:

```bash
f1 sim --no-agents
```

Open another host terminal, enter the same container, and run autonomy:

```bash
./dk.sh enter
f1 auto
```

Press `Ctrl+C` in either terminal to stop that ROS launch and remain inside the
container. In GPU mode RViz is a host window. In `--cpu` mode, open
`http://localhost:8080/vnc.html` for RViz. Useful host commands:

```bash
./dk.sh up       # Start Docker in default GPU mode without entering it.
./dk.sh up --cpu # Start CPU/noVNC mode without entering it.
./dk.sh enter    # Open another shell in the running container.
./dk.sh status   # Inspect containers and ROS launches.
./dk.sh stop     # Stop containers but preserve ROS build volumes.
```

The expected local checkout is:

```bash
~/f1tenth_dev
```

The simulator package should be here:

```bash
~/f1tenth_dev/f1tenth_gym_ros
```

The current working assumption is that the related packages are checked out as sibling folders under `~/f1tenth_dev`, for example:

```text
~/f1tenth_dev/f1tenth_gym_ros
~/f1tenth_dev/centerline_tools
~/f1tenth_dev/path_following_v2
~/f1tenth_dev/rl_speed_inference
~/f1tenth_dev/rl_training
~/f1tenth_dev/particle_filter
~/f1tenth_dev/range_libc
~/f1tenth_dev/slam_toolbox
~/f1tenth_dev/reactive_control/safety_bubble
~/f1tenth_dev/path_following/boundary_detection
~/f1tenth_dev/path_following/path_following
~/f1tenth_dev/drive_arbitration
```

More general setup and dependency installation instructions should stay in:

```bash
~/f1tenth_dev/README.md
```

---

# 1. Start and enter the Docker container

Current Docker image:

```text
f1tenth_gym_ros_localization_ready:latest
```

## 1.1 Docker Compose way, without NVIDIA GPU support

Container name:

```text
f1tenth_gym_ros_loc
```

Go to the simulator package:

```bash
cd ~/f1tenth_dev/f1tenth_gym_ros
```

Start the container and noVNC service:

```bash
docker compose -f docker-compose_loc.yml up
```

If your system uses the older Compose V1 command, use:

```bash
docker-compose -f docker-compose_loc.yml up
```

Enter the running container from another terminal:

```bash
docker exec -it f1tenth_gym_ros_loc /bin/bash
```

The noVNC browser display is usually available at:

```text
http://localhost:8080
```

## 1.2 Rocker way, with NVIDIA GPU and X11 support

Container name:

```text
f1tenth_gym_ros_rocker
```

Go to the simulator package:

```bash
cd ~/f1tenth_dev/f1tenth_gym_ros
```

Make sure the script is executable:

```bash
chmod +x f1tenth_rocker_loc_start.sh
```

Start the container:

```bash
./f1tenth_rocker_loc_start.sh
```

Enter the running container from another terminal:

```bash
docker exec -it f1tenth_gym_ros_rocker /bin/bash
```

---

# 2. Basic ROS 2 commands inside the container

In every new container terminal, source ROS 2 Foxy first:

```bash
source /opt/ros/foxy/setup.bash
```

After building packages, also source the workspace:

```bash
source /sim_ws/install/local_setup.bash
```

Useful workspace location:

```bash
cd /sim_ws
```

---

# 3. Current main flow: Raceline Path Following with RViz2 simulator

Run the following steps in order.

The current runtime flow is:

```text
centerline_tools/raceline_publisher
        ↓
/raceline_waypoints
        ↓
path_following_v2/path_generator
        ├── /path_following_v2/local_path
        └── /path_following_v2/rule_speed_index

optional rl_speed_inference/ppo_speed_node
        └── /rl_speed_inference/speed_residual_mps

path_following_v2/path_following_v2
        ↓
/drive
```

## 3.1 Build and start the RViz2 simulator, true-location-known version

Terminal 1, inside container:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select f1tenth_gym_ros
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

This starts the simulator/RViz side.

## 3.2 Build and start `centerline_tools`, global raceline publisher

Terminal 2, inside container:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select centerline_tools
source install/local_setup.bash
ros2 launch centerline_tools raceline_publisher_sim_launch.py
```
or if want to launch the raceline in the reverse direction:
```bash
ros2 launch centerline_tools raceline_publisher_sim_launch.py direction:=reverse
```

Expected important topics:

```text
/raceline_path
/raceline_markers
/raceline_waypoints
```

Quick check:

```bash
ros2 topic echo /raceline_waypoints --once
```

## 3.3 Build and start `path_following_v2`

Terminal 3, inside container:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select path_following_v2
source install/local_setup.bash
ros2 launch path_following_v2 path_following_v2_sim_launch.py
```

Expected important topics:

```text
/path_following_v2/local_path
/path_following_v2/rule_speed_index
/drive
```

Quick checks:

```bash
ros2 topic echo /path_following_v2/local_path --once
ros2 topic echo /path_following_v2/rule_speed_index --once
ros2 topic echo /drive --once
```

## 3.4 Speed mode in `path_following_v2`

The speed mode is configured in:

```bash
/sim_ws/src/path_following_v2/config/path_following_v2_sim.yaml
```

Use rule-based speed only:

```yaml
speed_mode: 0
```

Use rule-based speed plus RL residual when available:

```yaml
speed_mode: 1
```

Behavior of `speed_mode: 1`:

```text
if /rl_speed_inference/speed_residual_mps is fresh:
    final_speed = rule_speed + residual
else:
    final_speed = rule_speed
```

So if the RL inference node is not started, killed, or stale, the follower should fall back to pure rule-based speed.

Current simulator speed settings:

```yaml
speed_min: 1.0
speed_max: 10.0
rule_min_speed_mps: 1.0
rule_max_speed_mps: 6.0
max_speed_delta_per_step_mps: 0.2
```

## 3.5 Build and start `rl_speed_inference`, optional RL speed residual

Only needed when `path_following_v2_sim.yaml` has:

```yaml
speed_mode: 1
```

Terminal 4, inside container:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select rl_speed_inference
source install/local_setup.bash
ros2 launch rl_speed_inference rl_speed_inference_sim_launch.py
```

Expected output topic:

```text
/rl_speed_inference/speed_residual_mps
```

Quick check:

```bash
ros2 topic echo /rl_speed_inference/speed_residual_mps --once
```

Important file to check before running:

```bash
/sim_ws/src/rl_speed_inference/config/rl_speed_inference_sim.yaml
```

Make sure these paths exist and match your current setup:

```yaml
model_path: "/sim_ws/src/rl_training/models/ppo_speed_spielberg_section_reward_v2_1000k_continued.zip"
centerline_csv: "/sim_ws/src/centerline_tools/centerline_output/raceline_points_smooth.csv"
```

---

# 4. Recommended launch order

For rule-based-only raceline following:

```text
1. f1tenth_gym_ros simulator
2. centerline_tools raceline publisher
3. path_following_v2 with speed_mode: 0
```

For RL-boosted raceline following:

```text
1. f1tenth_gym_ros simulator
2. centerline_tools raceline publisher
3. path_following_v2 with speed_mode: 1
4. rl_speed_inference
```

In `speed_mode: 1`, it is okay to start `path_following_v2` before `rl_speed_inference`. The car should use rule-based speed first and switch to rule + residual after the residual topic becomes fresh.

---

# 5. Offline tools inside the container

## 5.1 Centerline and raceline generation

Workspace:

```bash
cd /sim_ws/src/centerline_tools
```

Please see the local package README for detailed instructions:

```bash
cat README.md
```

The validated IFAC Roboracer raceline selected by `f1 auto` is:

```bash
/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
```

The generic offline generator writes new, inactive output to:

```bash
/sim_ws/src/centerline_tools/centerline_output/raceline_points_smooth.csv
```

Expected CSV columns:

```text
index,x,y,yaw,curvature,curvature_abs
```

## 5.2 RL training and validation

Workspace:

```bash
cd /sim_ws/src/rl_training
```

Please see the local package README for detailed instructions:

```bash
cat README.md
```

The runtime inference package does not need `rl_training` to be built as a ROS package, but it does need the trained model `.zip` file.

## 5.3 Obstacle map creation

Workspace:

```bash
cd /sim_ws/src/centerline_tools
```

The obstacle creation tool allows one or more square obstacles to be manually placed along the generated centerline for obstacle detection and avoidance testing.

Run:

```bash
python3 obstacle_creation_tool.py \
    Spielberg_map.png \
    Spielberg_map.yaml
```

By default, the tool uses:

```bash
centerline_output/centerline_points_smooth.csv
```

The UI allows:

* selecting `left` or `right` relative to the centerline driving direction;
* setting the desired boundary gap in meters;
* setting the square obstacle size in meters;
* clicking near the centerline to add obstacles;
* undoing, resetting, and saving the generated obstacle map.

The main configurable constraints are defined near the top of:

```bash
obstacle_creation_tool.py
```

Current defaults include:

```bash
MIN_BOUNDARY_GAP_M = 0.50
MAX_OBSTACLE_SIZE_M = 0.50
```

The generated simulator map files are written by default to:

```bash
obstacle_output/
```

For example:

```bash
obstacle_output/
├── Spielberg_map_obstacles.png
├── Spielberg_map_obstacles.yaml
├── Spielberg_map_obstacles_clearance.csv
└── Spielberg_map_obstacles_debug.png
```

The essential files used by the simulator map server are:

```bash
Spielberg_map_obstacles.png
Spielberg_map_obstacles.yaml
```

The original map files are not overwritten.

---

# 6. Useful cleanup and rebuild commands

Clean and rebuild one package:

```bash
cd /sim_ws
rm -rf build/<package_name> install/<package_name> log
source /opt/ros/foxy/setup.bash
colcon build --packages-select <package_name>
source install/local_setup.bash
```

Example:

```bash
cd /sim_ws
rm -rf build/path_following_v2 install/path_following_v2 log
source /opt/ros/foxy/setup.bash
colcon build --packages-select path_following_v2
source install/local_setup.bash
```

Check active ROS graph:

```bash
ros2 node list
ros2 topic list
```

Check topic frequency:

```bash
ros2 topic hz /drive
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map ego_racecar/base_link
```

---

# 7. Current important packages

```text
f1tenth_gym_ros       simulator bridge and RViz launch
centerline_tools      offline raceline generation and runtime raceline publisher
path_following_v2     local path extraction, rule speed, pure-pursuit control
rl_speed_inference    PPO residual speed inference node
rl_training           offline PPO speed training and validation
particle_filter       localization package
range_libc            dependency for particle filter
```
