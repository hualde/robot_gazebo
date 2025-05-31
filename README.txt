
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/robot_gazebo/urdf/differential_robot.urdf.xacro)"
ros2 run ros_gz_sim create -topic robot_description -name my_robot1 -z 0.1
ros2 launch ros_gz_sim gz_sim.launch.py
