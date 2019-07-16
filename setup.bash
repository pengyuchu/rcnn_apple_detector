# Run it before rosrun

CATKIN_WORKSPACE=/home/rival/Documents/Repos/ROS

source $CATKIN_WORKSPACE/devel/setup.bash
export CMAKE_PREFIX_PATH=$CATKIN_WORKSPACE/devel:/opt/ros/kinetic
export ROS_PACKAGE_PATH=$CATKIN_WORKSPACE/src:/opt/ros/kinetic/share
