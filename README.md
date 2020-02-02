graphslam_ros2
====
this is a ros2 slam package of the frontend using gicp/ndt scan matching and the backend using graph-based slam. 

## io
- input  
/initial_pose  (geometry_msgs/PoseStamed)  
/input_cloud  (sensor_msgs/PointCloud2)  
/tf(from "base_link" to LiDAR's frame) 
- output  
/current_pose (geometry_msgs/PoseStamped)  
/map  (sensor_msgs/PointCloud2)  
/tf(from "map" to "base_link")  

## params

- frontend 

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT"|"NDT" or "GICP"|
|ndt_resolution|double|5.0|resolution size length of voxels[m]|
|voxel_leaf_size|double|0.2|down sample size of input cloud[m]|
|trans_for_mapupdate|double|1.5|moving distance of map update[m]|

- backend(Unimplemented yet) 

## demo
### frontend only
demo data(ROS1) by Autoware Foundation

```
wget http://db3.ertl.jp/autoware/sample_data/sample_moriyama_150324.tar.gz
tar zxfv sample_moriyama_150324.tar.gz
```

```
ros2 launch scanmatcher mapping.launch.py
```

```
ros2 topic pub initial_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 0, y: 0}, orientation: {z: 0, w: 1}}}' --once
```

```
ros2 bag play -s rosbag_v2 sample_moriyama_150324.bag 
```

<img src="./scanmatcher/images/mapping.png" width="640px">

### frontend and backend(Unimplemented yet)
demo data(ROS1) by Meiji University[https://drive.google.com/file/d/1VEy_iJZKEGcNDDKsK-YrqgKxwy6qsric/view]



~~Note)To use this rosbag, You need to git clone [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/154b35b6f960a8fc782de27c1b2b0c903baac4b3) ,
add `my_mapping_rules.yaml` in **ros1_bridge**'s folder,~~
```
-
    ros1_package_name: 'velodyne_msgs'
    ros2_package_name: 'velodyne_msgs'
```
~~and modifiy package.xml and CMakeLists.txt in ros1_bridge in the reference to [this document](https://github.com/ros2/ros1_bridge/blob/154b35b6f960a8fc782de27c1b2b0c903baac4b3/doc/index.rst).~~



```
ros2 run graphslam_main graphslam_node 
```

```
ros2 bag play -s rosbag_v2 infant_outdoor.bag 
```

## Used Libraries 

- Eigen
- PCL(BSD3)
- ceres-solver/g2o
