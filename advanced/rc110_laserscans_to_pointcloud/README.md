# RC 1/10 Scan Fuse

Combines two lidar scans into one point cloud.

Yet without motion compensation, so slow car movement is required.

## Subscribed Topics

```
/scan_1 [sensor_msgs::LaserScan]
    2D lidar scan 1
    
/scan_2 [sensor_msgs::LaserScan]
    2D lidar scan 2
```

## Published Topic

```
/lidar_cloud [sensor_msgs::PointCloud2]
    3D point cloud combined from scan_1 and scan_2
```

## Parameters

```
base_frame (string, default: base_link)
    base frame id

lidar_frame_1 (string, default: front_lidar)
    lidar 1 frame id
    
lidar_frame_2 (string, default: rear_lidar)
    lidar 2 frame id
```