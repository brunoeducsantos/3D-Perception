#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    cloud_msg = ros_to_pcl(pcl_msg)
    pcl.save(cloud_msg, "1.pcd")     
    # Statistical Outlier Filtering
    outlier_filter = cloud_msg.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filter_out = outlier_filter.filter()
    pcl.save(cloud_filter_out, "2.pcd")
    # Voxel Grid Downsampling
    vox = cloud_filter_out.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)    
    cloud_vox = vox.filter()
    pcl.save(cloud_vox, "3.pcd")

    # PassThrough Filter
    passthrough = cloud_vox.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filter_pass = passthrough.filter()
    
    passthrough2 = cloud_filter_pass.make_passthrough_filter()
    filter_axis = 'x'
    passthrough2.set_filter_field_name(filter_axis)
    axis_min = 0.4
    axis_max = 2.5
    passthrough2.set_filter_limits(axis_min, axis_max)
    cloud_filter2_pass = passthrough2.filter()
    pcl.save(cloud_filter2_pass, "4.pcd")
   
    #  RANSAC Plane Segmentation
    max_distance = 0.04
    seg = cloud_filter2_pass.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    #  Extract inliers and outliers
    cloud_objects = cloud_filter2_pass.extract(inliers, negative=True)
    pcl.save(cloud_objects, "file5.pcd")

    #  Euclidean Clustering
    white_cloud =XYZRGB_to_XYZ(cloud_objects) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.027)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
      for i, indice in enumerate(indices):
        color_cluster_point_list.append([white_cloud[indice][0],
                                        white_cloud[indice][1],
                                        white_cloud[indice][2],
                                        rgb_to_float(cluster_color[j])])
    
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)    
    pcl.save(cluster_cloud, "cloud.pcd")    
 
    # Convert PCL data to ROS messages
    ros_cloud_objects =  pcl_to_ros(cloud_objects) 
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_ros_seg.publish(ros_cluster_cloud)

    # Classify the clusters
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # Convert  pcl to ROS message 
        ros_cluster =  pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))        
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .2
        object_markers_pub.publish(make_label(label,label_pos, index))
         
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

        # Publish the list of detected objects
    
        detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
      pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
      pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    object_name =  String()
    object_group =  String()
    pick_pose = Pose()
    place_pose = Pose()
    test_scene_num = Int32()
    arm_name = String()
    dict_list = []
    
    #  Get/Read parameters
    object_list_param = rospy.get_param('/object_list') 
    dropbox_list_param =  rospy.get_param('/dropbox') 
    # Parse parameters into individual variables
    test_scene_num.data = 3
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    #  Loop through the objects
     
    for object in object_list:	
        # Get the PointCloud for a given object and obtain it's centroid
	label = object.label
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroid = [np.asscalar(i) for i in np.mean(points_arr, axis=0)[:3]]
        pick_pose.position.x = centroid[0]
	pick_pose.position.y = centroid[1]
	pick_pose.position.z = centroid[2]

        # Find the group associated with the object
        for i in range(len(object_list_param)):
            if (object_list_param[i]['name'] == label):
		object_group.data = object_list_param[i]['group']
                object_name.data= object_list_param[i]['name']

        # Loop through the dropbox parameters list
        for i in range(len(dropbox_list_param)):
            if (dropbox_list_param[i]['group'] == object_group.data):
               # Create 'place_pose' for the object
               place_pose.position.x = dropbox_list_param[i]['position'][0]
	       place_pose.position.y = dropbox_list_param[i]['position'][1]
	       place_pose.position.z = dropbox_list_param[i]['position'][2]   
               # Assign the arm to be used for pick_place
	       arm_name.data = dropbox_list_param[i]['name'] 
        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_req = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_req)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
           pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert  message variables to be sent as a service request
           resp = pick_place_routine(test_scene_num , object_name , arm_name , pick_pose , place_pose )

           print ("Response: ",resp.success)

        except rospy.ServiceException, e:
           print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml("output_3.yaml",dict_list)  


if __name__ == '__main__':

    #  ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers 
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_ros_seg = rospy.Publisher("/pcl_cloud", PointCloud2, queue_size=1)
    object_markers_pub  = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub= rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)  
    
    #  Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    #Spin while node is not shutdown
    while not rospy.is_shutdown():
    	rospy.spin()

