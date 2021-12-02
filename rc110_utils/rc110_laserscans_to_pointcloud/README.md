# RC 1/10 Scan Fuse

Combines two lidar scans into one point cloud.

Yet without motion compensation, so slow car movement is required.

## Subscribed Topics

```
/front_scan [sensor_msgs::LaserScan]
    2D lidar scan 1
    
/rear_scan [sensor_msgs::LaserScan]
    2D lidar scan 2
```

## Published Topic

```
/points2 [sensor_msgs::PointCloud2]
    3D point cloud combined from front_scan and rear_scan
```

## Parameters

```
base_frame (string, default: base_link)
    base frame id

front_lidar_frame (string, default: front_lidar)
    lidar 1 frame id
    
rear_lidar_frame (string, default: rear_lidar)
    lidar 2 frame id
```