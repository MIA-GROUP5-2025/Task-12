# Task-12.1 | Positional Control

## Requirements
  * ROS Noetic
  * TurtleBot3 Packages
  * Gazebo

## Steps
### Make Turtlebot move in any position autonomously using PID controller
 1. **Install ROS and TurtleBot3 Packages**
    ```bash
    sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
 2. **Set Up Your Workspace**
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
 3. **Make a new package and write the PID publisher and subscriber nodes**
    Or you can just download the pid folder present in this repo and run `catkin_make`
    ```bash
    cd ~/catkin_ws/src
    catkin_create_pkg pid rospy geometry_msgs std_msgs nav_msgs
    cd pid
    mkdir scripts
    cd scripts
    touch pid_pub.py
    touch pid_sub.py
    chmod +x pid_pub.py
    chmod +x pid_sub.py
 4. **Launch gazebo**
![launch](https://github.com/user-attachments/assets/9299084d-4dfe-4815-81d3-575bcfb08e6b)
    ```bash
    export TURTLEBOT3_MODEL=burger
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
 5. **In another terminal, run the publisher node**
    ```bash
    rosrun pid pid_pub.py
 6. **In another terminal, run the subscriber node**
    ```bash
    rosrun pid pid_sub.py
 7. **In the publisher node's terminal you can write your desired coordinates to the robot**
    ![pub](https://github.com/user-attachments/assets/07b94e06-9095-4df8-b0ac-52f1fd330d12)

 8. **You can edit kp, kd, and ki values in the subscriber node to see how they affect the robot's movement**
