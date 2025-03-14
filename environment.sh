source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
# export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/moos-dawg-2024/Firmware
# . ~/moos-dawg-2024/Firmware/Tools/setup_gazebo.bash ~/moos-dawg-2024/Firmware ~/moos-dawg-2024/Firmware/build/px4_sitl_default

if [ $# -gt 0 ]; then
	export ROS_MASTER_IP=$1
    echo "ROS_MASTER_IP set to $ROS_MASTER_IP"
    source set_ros_master.sh $ROS_MASTER_IP
else
    source set_ros_master.sh 127.0.0.1
fi

if [ $# -gt 0 ]; then
	export ROS_IP=$2
    echo "ROS_IP set to $ROS_IP"
    source set_ros_ip.sh $ROS_IP
else
    source set_ros_ip.sh 127.0.0.1
fi

# export PYTHONPATH=$PYTHONPATH:$HOME/duckiepond_gazebo/catkin_ws/devel/lib/python2.7/dist-packages
# export PATH=$PATH:$HOME/moos-dawg-2024/svn-mirror/bin
# export MOOS_DIR="$HOME/moos-dawg-2024/svn-mirror"
# export PATH=$PATH:$HOME/duckiepond-nctu/moos-ivp-taiwanMoos/bin
# export PATH=$PATH:$HOME/moos-dawg-2024/moos/bin
export MOOS_DIR=/home/moos-dawg/moos-dawg-2024/svn-mirror/build/MOOS/MOOSCore