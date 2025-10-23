"""
1.0 OS for the Raspberry Pi 4
    - Ubuntu 22.04 LTS (64 bit)
1.1 Core Tools
    - numpy
    - matplotlib
    - pyproj
2.0 Install ROS 2 Humble and Nav 2
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe
        sudo apt update
        sudo apt install -y ros-humble-desktop
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        source ~/.bashrc
2.1 Add Navigation 2 and localization
        sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
                    ros-humble-robot-localization ros-humble-slam-toolbox
2.2 Check (should print messages)
        ros2 run demo_nodes_cpp talker
3.0 Create your ROS 2 workspace
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws
        colcon build
        echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
        source ~/.bashrc
4.0 Add your first Python package
        cd ~/ros2_ws/src
        ros2 pkg create --build-type ament_python rc_nav --dependencies rclpy geometry_msgs nav2_msgs
[NOTE] - In rc_nav/rc_nav/, google_rout_to_nav2.py and paste the Google-waypoint example from before.
4.1 Edit rc_nav/setup.py -> Add an entry point
        entry_points={
    'console_scripts': [
        'google_route_to_nav2 = rc_nav.google_route_to_nav2:main',
    ],
},
4.2 Rebuilt and source
        cd ~/ros2_ws
        colcon build --symlink-install
        source install/setup.bash
4.3 Run it
        ros2 run rc_nav google_route_to_nav2
5.0 Connect your RC motors
6.0 Add GPS + IMU + localization
        sudo apt install -y ros-humble-robot-localization ros-humble-gps-tools
[NOTE] - Run navsat_transform_node (fuses gps, odom, and imu), ekf_node (from robot localization), and Nav2's navigation_launch.py (for planning)
7.0 Integrate google maps route
[NOTE] - Use google directions API on laptop or motherboard to get a polyline (you can use python's requests and the api key)
7.1 Decode with snippet
        import polyline
        points = polyline.decode(encoded_string)  # gives [(lat, lon), ...]
[NOTE] Feed those lat/Ion pairs into the GOOGLE_WAYPOINTS list in your node.
7.2 Launch your Nav2 bring-up, then run your waypoint node
        ros2 launch nav2_bringup navigation_launch.py
        ros2 run rc_nav google_route_to_nav2
[Recommended]    
    - Make sure ROS 2 + RViz runs on the pi
    - Test /cmd_vel -> motors (manual teleop)
    - Run Nav2 with a dummy map or empty costmap
    - Run the waypoint node -> Watch RViz follow waypoints
    - Add GPS + IMU
    - Obstacle Detection with your cameras (OPENCV)
"""
