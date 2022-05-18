# extract lidar and images from rosbag and save timestamp-matched jpeg and pcd files 

## Introduction
rosbags contain different topics 
for lidar (p40) : /pandar40p_0/pandar_packets   : pandar_msgs/PandarScan
                  /pandar40p_2/pandar_packets   : pandar_msgs/PandarScan
for images : /dev/video0/compressed      : sensor_msgs/CompressedImage   
             /dev/video1/compressed      : sensor_msgs/CompressedImage   
             /dev/video2/compressed      : sensor_msgs/CompressedImage   
             /dev/video3/compressed      : sensor_msgs/CompressedImage   
             /dev/video4/compressed      : sensor_msgs/CompressedImage   
             /dev/video5/compressed      : sensor_msgs/CompressedImage   
             /dev/video6/compressed      : sensor_msgs/CompressedImage   
             /dev/video7/compressed      : sensor_msgs/CompressedImage
we use rosbag module to extract and save pcd file, so before extracting, transform pandar_packets to pandar_pointcloud. Tool: pandar_converter. Fold: pandar. 
 

## Requirements
All codes are tested under the following environment:
*   Ubuntu
*   Python 3.7
*   roabag 
*   rospy
*   numpy



## run 
1. use `conda` to manage the environment:
```
conda create -n pcd python=3.7
```

2. activate environment:
```
conda activate pcd
```

3. extract infomations:

```
./toe.sh
```


## result-like 

       FoldName             Description
----------------------------------------------------------------------------
   1    pcd_all             pcdfile 
   
   2    camera_f60         images for front FOV60  

   3    camera_f120        images for front FOV120

   4    camera_l60         images for left FOV60
   
   5    camera_l120        images for left FOV120

   6    camera_r60         images for right FOV60

   7    camera_r120        images for right FOV120



