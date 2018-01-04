# 3D-Perception
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

This project aims to create a node and several topics to a rgb-d camera node in ROS to perform a perception pipeline of collected point cloud containing a series of objects in a [ROS](http://www.ros.org/)  environment.
The ROS [ROS](http://www.ros.org/) setup environment includes:

- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot

In addition, this project aims to label the perceived objects and pick them using PR2 robotic arms. After picking the object , it will be placed in the closest dropbox. 


# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```
# Usage 
To run the the project, you should select first one of three scenarios in *pick_place_project.launch* file and launchg gazebo as follows: 
```
$ roslaunch pr2_robot pick_place_project.launch
```

Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


Afterwards, to run the pipeline and generate the labels for each object you run the [3dperception.py](https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/pr2_robot/scripts/3dperception.py) script:
```
$ rosrun pr2_robot 3dperception.py
```
Finally, to generate the data to traing SVM model you should also clone [sensor_stick](https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/sensor_stick) ROS package as follows:
```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/sensor_stick
$ cd ~/catkin/
$ catkin_make
$ source devel/setting.bash
$roslaunch sensor_stick training.launch
$rosrun sensor_stick capture_features.py
```


# Disclamer
This project was cloned from [Udacity perception project](https://github.com/udacity/RoboND-Perception-Project) in the context of [Robotics Software Engineer nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209).

# TODO
- Create a collision map of objects on toptable
- Finetune pick and place trajectory


