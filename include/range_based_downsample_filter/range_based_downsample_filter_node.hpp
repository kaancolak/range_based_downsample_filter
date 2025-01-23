#ifndef RANGE_BASED_DOWNSAMPLE_FILTER__RANGE_BASED_DOWNSAMPLE_FILTER_NODE_HPP_
#define RANGE_BASED_DOWNSAMPLE_FILTER__RANGE_BASED_DOWNSAMPLE_FILTER_NODE_HPP_

#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include "autoware_point_types/types.hpp"

#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/transform_info.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <vector>

namespace range_based_downsample_filter
{

using autoware_point_types::PointXYZIRC;  

struct StoredPointCloud {
  pcl::PointCloud<PointXYZIRC>::Ptr cloud;
  std_msgs::msg::Header header;
};

class RangeBasedDownsampleFilterNode : public pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  // TODO(atsushi421): Temporary Implementation: Remove this interface when all the filter nodes
  // conform to new API
  //virtual void faster_filter(
  //  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  //  const TransformInfo & transform_info);
private:
  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;

  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit RangeBasedDownsampleFilterNode(const rclcpp::NodeOptions & options);
};
}  // namespace range_based_downsample_filter

#endif  // POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_