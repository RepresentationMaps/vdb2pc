// Copyright 2025 PAL Robotics, S.L.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


/*********************************************************************
 *
 * Software License Agreement
 *
 * Copyright (c) 2025, PAL Robotics, S.L.
 * All rights reserved.
 *
 * This file is part of the vdb2pc library,
 * derived from spatio_temporal_voxel_layer
 * (https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/ros2).
 * Original work Copyright (c) 2018, Simbe Robotics, Inc.
 *
 * Modifications Copyright (c) 2025, PAL Robotics, S.L.
 *
 * This software is licensed under the terms of the LGPLv3 license.
 * See the LICENSE file in the project root for more information.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors: Lorenzo Ferrini, Jozsef Palmieri
 * Purpose: convert VDB structures into PCL pointclouds (and vicecversa)
 *
 *********************************************************************/

#ifndef VDB2PC__VDB2PC_PUBLISHER_HPP_
#define VDB2PC__VDB2PC_PUBLISHER_HPP_

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "vdb2pc.hpp"

namespace vdb2pc
{
namespace ros_utils
{
template<class T>
class VDB2PCPublisher
{
private:
  const std::shared_ptr<rclcpp::Node> node_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  std::string reference_frame_;

public:
  VDB2PCPublisher(
    const std::shared_ptr<rclcpp::Node> & node_handler,
    const std::string & topic_name,
    const std::string & reference_frame)
  : node_handler_(node_handler),
    pc_publisher_(node_handler_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 1)),
    reference_frame_(reference_frame) {}

  ~VDB2PCPublisher() {}

  void publish(T & grid)
  {
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    sensor_msgs::msg::PointCloud2 point_cloud_msg;

    vdb2pc::transform(grid, point_cloud);
    point_cloud.height = 1;
    point_cloud.width = point_cloud.size();

    pcl::toROSMsg(point_cloud, point_cloud_msg);

    point_cloud_msg.header.stamp = node_handler_->now();
    point_cloud_msg.header.frame_id = reference_frame_;

    pc_publisher_->publish(point_cloud_msg);
  }

  void updateReferenceFrame(const std::string & reference_frame)
  {
    reference_frame_ = reference_frame;
  }
};
}      // namespace ros_utils
}  // namespace vdb2pc
#endif  // VDB2PC__VDB2PC_PUBLISHER_HPP_
