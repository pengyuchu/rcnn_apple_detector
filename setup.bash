# Run it before rosrun

CATKIN_WORKSPACE=/home/bizon/catkin_workspace

source $CATKIN_WORKSPACE/devel/setup.bash
export CMAKE_PREFIX_PATH=$CATKIN_WORKSPACE/devel:/opt/ros/melodic
export ROS_PACKAGE_PATH=$CATKIN_WORKSPACE/src:/opt/ros/melodic/share
