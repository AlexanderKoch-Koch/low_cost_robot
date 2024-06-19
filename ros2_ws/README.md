# ros2_ws

## About
- This workspace is a directory containing the `ROS 2` packages for the `low_cost_robot` project.  
- Before using these `ROS 2` packages, it’s necessary to source the `ROS 2` installation for this workspace in the terminal where you plan to work.  
- Since `ROS 2` is more natively supported on the `Ubuntu` platform, all instructions and commands in the `README.md` under the `ros2_ws` directory are for Ubuntu. If you use the `MacOS` platform, instructions and commands may be different.

## `ROS 2` Package List in This Workspace
1. **low_cost_robot_description**: This package is for the URDF of the robot arm (follower).

## Prerequisites
1. `ROS 2` installed (You can refer to this [document](https://docs.ros.org/en/humble/Installation.html) to install `ROS 2` Humble Hawksbill or another `ROS 2` [distribution](https://docs.ros.org/en/humble/Releases.html).)
2. `colcon` installed (You can refer to this [document](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) to install `colcon`.)
3. `rosdep` installed (You can refer to this [document](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) to install `rosdep`.)


## Installation
1. Source the ROS 2 setup files ([reference](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#id4)):
   ```shell
   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: setup.bash, setup.sh, setup.zsh
   # Replace "humble" with your ROS 2 distribution if you're not using "ROS 2 Humble Hawksbill"
   source /opt/ros/humble/setup.bash
   ```
2. Change directory to `ros2_ws` and install the dependencies by `rosdep` of `ROS 2` packages ([reference](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)):
   ```shell
   # Assume the current directory is `low_cost_robot`
   cd ros2_ws

   # `--ignore-src` means to ignore installing dependencies
   # Use `rosdep -h` to get help
   rosdep install --from-paths src -y --ignore-src
   ```
3. Build the workspace by `colcon` ([reference](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)):
   ```shell
   # The option `--symlink-install` allows the installed files to be changed by
   # changing the files in the source space (e.g. Python files or other non-compiled resources)
   # for faster iteration.
   colcon build --symlink-install
   ```

4. Source the installation of this workspace ([reference](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)):
   ```shell
   # Assume the current directory is `low_cost_robot`
   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: local_setup.bash, local_setup.sh, local_setup.zsh
   source install/local_setup.bash
   ```

- > Notice:  
   > Normally, you need to source the ROS 2 setup files and the installation of this workspace after opening the terminal:
   > ```shell
   > # Replace ".bash" with your shell if you're not using bash
   > # Possible values are: setup.bash, setup.sh, setup.zsh
   > # Replace "humble" with your ROS 2 distribution if you're not using "ROS 2 Humble Hawksbill"
   > source /opt/ros/humble/setup.bash
   >
   > # Assume the current directory is `low_cost_robot`
   > cd ros2_ws
   >
   > # Replace ".bash" with your shell if you're not using bash
   > # Possible values are: local_setup.bash, local_setup.sh, local_setup.zsh
   > source install/local_setup.bash
   > ```