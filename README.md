# Range Based Downsample Filter

Downsamples point cloud data outside a specified region of interest (ROI) while preserving the original point density within the ROI.

## Overview

The range_based_downsample_filter package provides selective downsampling of LiDAR point clouds, preserving point density in important regions while reducing data in dense areas.

## Input/Output

| Name     | Type                    | Description                                |
|----------|------------------------|--------------------------------------------|
| `input`  | `sensor_msgs/PointCloud2` | Input point cloud from LiDAR              |
| `output` | `sensor_msgs/PointCloud2` | Downsampled point cloud                   |

## Parameters

### Core Parameters

| Name          | Type   | Default Value | Description                                      |
|--------------|--------|---------------|--------------------------------------------------|
| `voxel_size_x`| float  | 0.3           | Voxel leaf size for x dimension                 |
| `voxel_size_y`| float  | 0.3           | Voxel leaf size for y dimension                 |
| `voxel_size_z`| float  | 0.1           | Voxel leaf size for z dimension                 |
| `x_min`      | float  | 80.0          | Minimum x coordinate of ROI in meters           |
| `x_max`      | float  | 200.0         | Maximum x coordinate of ROI in meters           |
| `y_min`      | float  | -20.0         | Minimum y coordinate of ROI in meters           |
| `y_max`      | float  | 20.0          | Maximum y coordinate of ROI in meters           |
