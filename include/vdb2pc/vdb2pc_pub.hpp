/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
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
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
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
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 * Purpose: convert native vdb files to pointclouds
 *********************************************************************/
#ifndef VDB2PC_PUB_HPP_
#define VDB2PC_PUB_HPP_

#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "vdb2pc.hpp"

namespace ros_vdb2pc
{
    template<class T>
    class VDB2PCPublisher
    {
        private:
            const std::shared_ptr<rclcpp::Node> node_handler_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
            std::string reference_frame;

        public:
            VDB2PCPublisher(const std::shared_ptr<rclcpp::Node>& node_handler, const std::string& topic_name_, const std::string& reference_frame_) : node_handler_(node_handler)
            {
                this->pc_publisher_ = node_handler_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_,1);
                this->reference_frame = reference_frame_;
            }
           
            ~VDB2PCPublisher(){}

            void publish(T& grid_)
            {
                pcl::PointCloud<pcl::PointXYZI> point_cloud_;
                sensor_msgs::msg::PointCloud2 point_cloud_msg_;

                vdb2pc::transform(grid_,point_cloud_);
                point_cloud_.height = 1;
                point_cloud_.width = point_cloud_.size();

                pcl::toROSMsg(point_cloud_,point_cloud_msg_);

                point_cloud_msg_.header.stamp = node_handler_->now();
                point_cloud_msg_.header.frame_id = this->reference_frame;

                this->pc_publisher_->publish(point_cloud_msg_);
            }

            void updateReferenceFrame(const std::string& reference_frame_)
            {
                this->reference_frame = reference_frame_;
            }
    };
}; // namespace ros_vdb2pc
#endif // VDB2PC_PUB_HPP_