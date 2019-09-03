# rcnn_apple_detector pkg
> MSU apple detection project. 
> Detection module.

### Env requirements: 

* Ros: __Medolic__ for Ubuntu 18 or __Kinetic__ for Ubuntu 16
* Python: __Python3__
* Network: MSU EGR Network

### Prerequisites: 

1️⃣ Packages

Build __cv_bridge__ for Python3

```shell
# `python-catkin-tools` is needed for catkin tool
# `python3-dev` and `python3-catkin-pkg-modules` is needed to build cv_bridge
# `python3-numpy` and `python3-yaml` is cv_bridge dependencies
# `ros-kinetic-cv-bridge` is needed to install a lot of cv_bridge deps. Probaply you already have it installed.
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml
# Create catkin workspace
mkdir catkin_ws
cd catkin_ws
# sudo apt-get install ros-kinetic-catkin python-catkin-pkg
mkdir src build devel logs
catkin init
# Instruct catkin to set cmake variables
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# Instruct catkin to install built packages into install place. It is $CATKIN_WORKSPACE/install folder
catkin config --install
# Clone cv_bridge src
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
# Find version of cv_bridge in your repository: your_version
# apt-cache show ros-kinetic-cv-bridge | grep Version
# Checkout right version in git repo
# cd src/vision_opencv/
# git checkout your_version
# cd ../../
# Build
catkin build cv_bridge
# if boost python3 errors, modify src/vision_opencv/cv_bridge/CMakeLists.txt 
# Follow https://stackoverflow.com/questions/49221565/unable-to-use-cv-bridge-with-ros-kinetic-and-python3

# Extend environment with new package
source install/setup.bash --extend

# Then validate it

python3
>>> from cv_bridge.boost.cv_bridge_boost import getCvType
>>> 
```

Install Mask RCNN

```shell
git clone https://github.com/matterport/Mask_RCNN.git
cd Mask_RCNN 
pip3 install -r requirements.txt
python3 setup.py install
```

2️⃣ Environment Setup

[Optional] If you use all of packages in the same device, you don't need to setup this.

```shell
# Ensure use the same master in the LAN.
echo 'export ROS_MASTER_URI=http:35.9.138.211//:11311/' >> ~/.bashrc 
echo 'export ROS_IP=`hostname -I`' >> ~/.bashrc 
```

### Install & Run

```shell
# enter your ros workspace
cd ~/catkin_ws/src
git clone https://github.com/bennie-msu/rcnn_apple_detector.git
cd ..
catkin build # It's not compatible with catkin_make
cd src/rcnn_apple_detector/weights

# download the weights model 
wget https://github.com/bennie-msu/rcnn_apple_detector/releases/download/Model/model.h5
cd .. # go back to the rcnn_apple_detector directionary
# Setup it when you open a new session or echo it into .bashrc
source setup.bash # You should edit it and modify CATKIN_WORKSPACE to yours
# pip3 install opencv-python
# Run in the rcnn_apple_detector dir
roscore # in another session
rosrun rcnn_apple_detector detection.py
# Make sure PYTHONPATH contains your system python3/package-sites (where you install opencv-python)
```

Now It's done 😊😊😊


### Protocol

| Topic       | Data Type                                                    | Description                                                  |
| ----------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| color_image | [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | Color images from RS camera                                  |
| depth_image | [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) | gray-scale depth image from RS camera                        |
| detections  | [Int32MultiArray](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int32MultiArray.html) | Bounding Boxes of apples detection in the form of (x1, y1, x2, y2) |
| positions   | [PoseArray](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html) | A sequence position data for each apple, organized in (x, y, z) |
