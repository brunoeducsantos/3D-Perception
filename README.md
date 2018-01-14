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
# Implementation

[point cloud after outliers removal]: https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/outliersremoval.PNG
[initial point cloud]:https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/initialPC.PNG
[voxel grid downsampling result]:https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/voxelgrid.PNG
[pass through filter result]: https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/pass.PNG
[RANSAC segmentation]:https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/ransac.PNG
[separate objects on the table]: https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/euclidean_clustering.PNG
[confusion matrix]:https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/normalizedconfusionmatrix.PNG
[label object 3]: https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/LabelObjects.PNG
[label object 2]:https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/labelobjects2.PNG
[label object 1]: https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/LabelObjects1.PNG
[output_1.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/output_1.yaml
[output_2.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/outputs/output_2.yaml
[output_3.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/output_3.yaml
[model]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/model.sav
The contributions to this project were developed in the following files:
* [Perception pipeline and labeling model](https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/PR2-PERCEPTION/pr2_robot/scripts/3dperception.py)
* [Features to train the model: colors and normals](https://github.com/BrunoEduardoCSantos/3D-Perception/blob/master/sensor_stick/scripts/capture_features.py)
In the following sections will be addressed the approach taken to:
* perform the perception pipeline
* generate features to training dataset  
* train the predictive model


### Pipeline to process input data cloud from RGB-D camera 
#### 1. Pipeline for filtering and RANSAC plane fitting implemented
During this first stage of input data cloud pre-processing the following filtering processes were applied:
* [Statistical outlier removal](http://pointclouds.org/documentation/tutorials/statistical_outlier.php)
* Voxel grid downsampling
* Pass through filtering
* Plane fitting using RANSAC

Outliers removal was applied to the cloud due to some points over the objects as we can tell from the [initial point cloud].

![alt_text][initial point cloud]

In order to remove the outliers we Gaussian distribution, all points whose mean distances are outside of an interval defined by the global distances mean+standard deviation are considered to be outliers and removed from the point cloud. Since the density was not large, the following parameters were considered to define the statistical outlier removal:
* mean_k = 50
* std_dev = 0.1

After outlier removal we got [point cloud after outliers removal].

![alt_text][point cloud after outliers removal]

The next step was applying a reduction of point cloud density to feed the train model later on . For this purpose, it was applied a voxel grid downsampling with the following parameter:
* leaf_size = 0.005 

After applying this process we obtain [voxel grid downsampling result]

![alt_text][voxel grid downsampling result]

The principal to define the density was based on the minimal amout of points to have a object shape.
The next step was applying filtering on z-axis to remove table and on x-axis to remove dropboxes side bound. 
For this purpose the following parameters were applied:
* z-axis : min=  0.6 ; max = 1.5
* x-axis: min = 0.4 ; max = 2.5 

After applying pass through filtering we obtain [pass through filter result].

![alt_text][pass through filter result]


The last step is fitting plane to extract objects from point cloud using RANSCAC, with a **max_dist = 0.04**. The small distance is due to density of  objects point cloud.
After this final step we obtain [RANSAC segmentation].

![alt_text][RANSAC segmentation]

#### 2. Pipeline including clustering for segmentation implemented.

For the purpose of having segmenation of objects on the table it was used a spatial clustering algorithm called Euclidean clustering. 
Euclidean clustering is based on a radius around a random point considering a minimum and maximum number of points to cluster a set of points. 
Regarding this concept, it was applied Euclidean clustering with the following parameters values:

* radius: 0.027
* min_cluster_size: 50
* max_cluster_size: 2000

After segmentation process we obtain [separate objects on the table]. 

![alt_text][separate objects on the table]

#### 2.  Features extracted and SVM trained &  Object recognition implementation

In order to predict the labels of the point cloud objects we used as features the **normals** and **colors** of associated to each point of object point cloud. After having the dataset for each object we train a SVM model to predict the its label. For this purpose a **linear kernel** with a tradoff parameter **C=0.55**. Training on our test dataset we obtain the following [confusion matrix].

![alt_text][confusion matrix]


### Pick and Place Setup

#### Object recognition for three test scenarios
After obtain our trained [model] , this one was applied on our pipeline applied to received images  being streamed from camera node in ROS.
As a result, we label the objects for the provided test scenarios, i.e., for different objects on the table (test1.world, test2.world, test3.world).

For test1.world we obtained the following labeled objects:

![alt_text][label object 1]

For test2.world we obtained:

![alt_text][label object 2]

Finally, for test3.world we obtained:

![alt_text][label object 3]

Using the existing ros node that communicates with my perception pipeline to perform sequential object recognition, we obtain requests into [output_1.yaml], [output_2.yaml], and [output_3.yaml] for each scene respectively.

# Disclamer
This project was cloned from [Udacity perception project](https://github.com/udacity/RoboND-Perception-Project) in the context of [Robotics Software Engineer nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209).

# TODO
- Create a collision map of objects on toptable
- Finetune pick and place trajectory


