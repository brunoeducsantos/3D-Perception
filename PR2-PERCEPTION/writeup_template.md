## Project: Perception Pick & Place
[point cloud after outliers removal]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outliersremoval.PNG
[initial point cloud]:https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/initialPC.PNG
[voxel grid downsampling result]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/voxelgrid.PNG
[pass through filter result]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/pass.PNG
[RANSAC segmentation]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/ransac.PNG
[separate objects on the table]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/euclidean_clustering.PNG
[confusion matrix]:https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/normalizedconfusionmatrix.PNG
[label object 3]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/LabelObjects.PNG
[label object 2]:https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/labelobjects2.PNG
[label object 1]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/LabelObjects1.PNG
[output_1.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/output_1.yaml
[output_2.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/outputs/output_2.yaml
[output_3.yaml]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/output_3.yaml
[model]: https://github.com/BrunoEduardoCSantos/3D-Perception/tree/master/PR2-PERCEPTION/outputs/model.sav
### Pipeline to process input data cloud from RGB-D camera 
#### 1. Pipeline for filtering and RANSAC plane fitting implemented
During this first stage of input data cloud pre-processing the following filtering processes were applied:
* Outliers removal
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

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further. 
Finally, some improvements on the code should be better outlier removal in order to improve our model prediction . In addition, a ROS node should have been created specific to object labeling. The next steps would be:
* create a new scenario 
* test my model on a new scenario  
* refine pick and place routine 
* improve logic conditions before arm pick the object




