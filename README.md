graphslam_ros2
====
this is a ros2 slam package of the frontend using gicp/ndt scan matching and the backend using graph-based slam. 

## io
- input  
/initial_pose  (geometry_msgs/PoseStamed)  
/input_cloud  (sensor_msgs/PointCloud2)  
- output  
/curent_pose (geometry_msgs/PoseStamped)  
/map  (sensor_msgs/PointCloud2)  

## params

- frontend 

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT"|"NDT" or "GICP"|
|ndt_resolution|double|1.0|resolution size length of voxels[m]|
|voxel_leaf_size|double|0.2|down sample size of a input cloud[m]|
|trans_for_update|double|1.5|moving distance of a map update[m]|

- backend 

## demo
demo data(ROS1) by Meiji University[https://drive.google.com/file/d/1VEy_iJZKEGcNDDKsK-YrqgKxwy6qsric/view]

```
ros2 run graphslam_main graphslam_node 
ros2 run scanmatcher scanmatcher_node
```

```
ros2 bag play -s rosbag_v2 infant_outdoor.bag 
```

## Used Libraries 

- Eigen
- PCL(BSD3)
- ceres-solver/g2o
