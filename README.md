# Goal navigation with ROS2 using A*

## Project Structure

### Own Additions
* `~/ros2_ws/src/Drone-RL-Control/drone_exploration` (ROS 2 package)
* `~/ros2_ws/maps` (directory)
* `~/ros2_ws/src/install_rtabmap.sh` (script)

### Modifications Made
* **`~/ros2_ws/src/Drone-RL-Control/siju_drone_description` package:** Modifications made to the `.xacro` file.
* **`~/ros2_ws/src/Drone-RL-Control/siju_drone_bringup/sjtu_drone_bringup`:** Modifications made to the spawn entity file.
* **`~/ros2_ws/src/Drone-RL-Control/siju_drone_bringup/config` folder:** Modifications made to the `.yaml` file.

---

## To Run the Project

1. Change directory to your ROS2 workspace:
   ```bash
   cd ~/ros2_ws
   ```

2. Build and source the workspace folder:
   ```bash
   colcon build
   source install/setup.bash
   ```

3. Launch the exploration nodes and wait roughly 90 seconds for the `rtabmap` process to spin up:
   ```bash
   ros2 launch drone_exploration exploration.launch.py
   ```

4. In a **second terminal**, start the Navigation2 lifecycle manager:
   ```bash
   ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:="[map_server]" -p autostart:=true
   ```

5. In a **third terminal**, load your workspace maps using the Nav2 map server:
   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/ros2_ws/src/maps/maps.yaml
   ```

6. In a **fourth terminal**, source the workspace configuration as shown in Step 2, then trigger the mission script:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch drone_exploration mission.launch.py
   ```

> 💡 **Note:** If Gazebo fails to display the environment map properly, press **Ctrl + C** in the execution terminal to kill the processes, then rerun the exploration launch script: `ros2 launch drone_exploration exploration.launch.py`.
