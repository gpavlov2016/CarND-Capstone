This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

# Team introduction
|              |     Name      | Location | LinkedIn | Image |
|--------------|---------------|----------|----------|--------------------------------|
| __Team Lead__| Guy Pavlov | San Jose, CA | [linkedin.com/in/guypavlov](https://linkedin.com/in/guypavlov) | <img src="./imgs/GuyPavlov.jpg" alt="Guy Pavlov" width="150" height="150"> |
|Member| Aaron | | | |
|Member| Jay |  |  |  |
|Member| Shubham | Santa Clara, CA | [linkedin.com/in/shubham1](https://linkedin.com/in/shubham1) | 

# Installation and Usage

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

# Architechture
This is the system architecture for this project, we have not worked on obstacle detection as part of this implementation


![image](imgs/1.png)


## Control
This module handles vehicle's throttle, brake and steering based on the provided list of waypoints to follow. We have used PID controller to handle these three components. 

Braking is done in the proportion of difference between current velocity and expected velocity. 

## Perception

## Planning
This module decide the vehicle path using various inputs like: vehicle's current position, velocity and location, state of various traffic lights on the way. This includes two main nodes: 
1. **Waypoint loader:** This loads the static waypoint data (CSV) and publishes to /base_waypoints 
 

 2. **Waypoint updater:** This is main planning node. This subscribes to three main topics, 1. current_pose, vehicle current position, 2. base_waypoints, list of all waypoints and 3. traffic_waypoint, location of traffic light. 
 
    If upcoming traffic light is red and vehicle is at stopping distance then system tries to stop the vehicle. In case of situation when there is no red light or vehicle is ahead of red light stop line, vehicle moves at max speed. 
 
# Results

Add video of the car in simulator here


# Known issues/ Future work