// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "range_based_downsample_filter/range_based_downsample_filter_node.hpp"

#include "pointcloud_preprocessor/downsample_filter/faster_voxel_grid_downsample_filter.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace range_based_downsample_filter
{
RangeBasedDownsampleFilterNode::RangeBasedDownsampleFilterNode(const rclcpp::NodeOptions & options)
: Filter("RangeBasecVoxelGridDownsampleFilter", options)
{
  // set initial parameters
  {
    voxel_size_x_ = static_cast<float>(declare_parameter("voxel_size_x", 0.3));
    voxel_size_y_ = static_cast<float>(declare_parameter("voxel_size_y", 0.3));
    voxel_size_z_ = static_cast<float>(declare_parameter("voxel_size_z", 0.1));
  
    // ROI parameters
    x_min_ = static_cast<float>(declare_parameter("x_min", 80.0));
    x_max_ = static_cast<float>(declare_parameter("x_max", 200.0));
    y_min_ = static_cast<float>(declare_parameter("y_min", -20.0));
    y_max_ = static_cast<float>(declare_parameter("y_max", 20.0));
  }

}

// TODO(atsushi421): Temporary Implementation: Delete this function definition when all the filter
// nodes conform to new API.
void RangeBasedDownsampleFilterNode::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);


  // Separate ROI and non-ROI points
  for (const auto & point : pcl_input->points) {
    if (point.x >= x_min_ && point.x <= x_max_ && 
        point.y >= y_min_ && point.y <= y_max_) {
      roi_points->points.push_back(point);
    } else {
      pcl_output->points.push_back(point);
    }
  }

  pcl_output->points.reserve(pcl_input->points.size());
  
  // Downsample non-ROI points
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_output);
  filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  filter.filter(*pcl_output);

  // Combine ROI and downsampled points
  *pcl_output += *roi_points;

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

// TODO(atsushi421): Temporary Implementation: Rename this function to `filter()` when all the
// filter nodes conform to new API. Then delete the old `filter()` defined above.
//void RangeBasedDownsampleFilterNode::faster_filter(
//  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
//  PointCloud2 & output, const TransformInfo & transform_info)
//{
//  std::scoped_lock lock(mutex_);
//  FasterVoxelGridDownsampleFilter faster_voxel_filter;
//  faster_voxel_filter.set_voxel_size(voxel_size_x_, voxel_size_y_, voxel_size_z_);
//  faster_voxel_filter.set_field_offsets(input, this->get_logger());
//  faster_voxel_filter.filter(input, output, transform_info, this->get_logger());
//}
}  

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(range_based_downsample_filter::RangeBasedDownsampleFilterNode)
