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

/* -------------------------------------------------------------------------- */
/*                                Header Files                                */
/* -------------------------------------------------------------------------- */

/* --------------------------------- Chrono --------------------------------- */
#include <chrono>
/* -------------------------------------------------------------------------- */

/* ---------------------------- PCL - Conversion ---------------------------- */
#include <pcl_conversions/pcl_conversions.h>
/* -------------------------------------------------------------------------- */

/* --------------------------- ROS Client Library --------------------------- */
#include <rclcpp/rclcpp.hpp>
/* -------------------------------------------------------------------------- */

/* ------------------------------ ROS Messages ------------------------------ */
#include <sensor_msgs/msg/point_cloud2.hpp>
/* -------------------------------------------------------------------------- */

/* --------------------------------- VDB2PC --------------------------------- */
#include "vdb2pc.hpp"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*                     VDB2PCPublisher - Class Definition                     */
/* -------------------------------------------------------------------------- */
namespace ros_vdb2pc
{
    template<class T>
    class VDB2PCPublisher : public rclcpp::Node
    {
        /* --------------------------------- Private -------------------------------- */
        private:

            /* ---------------------------------- Timer --------------------------------- */
            rclcpp::TimerBase::SharedPtr timer_;
            /* -------------------------------------------------------------------------- */

            /* ------------------------------- Publishers ------------------------------- */
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
            /* -------------------------------------------------------------------------- */

            /* ---------------------------------- Frame --------------------------------- */
            std::string reference_frame;
            /* -------------------------------------------------------------------------- */

            /* -------------------------------- Utilities ------------------------------- */
            vdb2pc::VDB2PointCloud<T> vdb_converter_;
            /* -------------------------------------------------------------------------- */

        /* -------------------------------------------------------------------------- */

        /* --------------------------------- Public --------------------------------- */
        public:
            /* ------------------------------- Constructor ------------------------------ */
            VDB2PCPublisher(const std::string& node_name_, const std::string& topic_name_, const std::string& reference_frame_) : Node(node_name_)
            {
                /* -------------------------- Publisher - Creation -------------------------- */
                this->pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_,1);
                /* -------------------------------------------------------------------------- */

                /* ------------------------ Point Cloud - Allocation ------------------------ */
                this->reference_frame = reference_frame_;
                /* -------------------------------------------------------------------------- */
            }
            /* -------------------------------------------------------------------------- */

            /* ------------------------------- Destructor ------------------------------- */
            ~VDB2PCPublisher()
            {
                
            }
            /* -------------------------------------------------------------------------- */

            /* -------------------------------- Utilities ------------------------------- */
            void publish(T& grid_)
            {
                /* --------------------------- Auxiliary Variables -------------------------- */
                pcl::PointCloud<pcl::PointXYZI> point_cloud_;
                sensor_msgs::msg::PointCloud2 point_cloud_msg_;
                /* -------------------------------------------------------------------------- */
                
                /* --------------------------------- VDB2PCL -------------------------------- */
                this->vdb_converter_.transform(grid_,point_cloud_);
                point_cloud_.height = 1;
                point_cloud_.width = point_cloud_.size();
                /* -------------------------------------------------------------------------- */

                /* ------------------------------- PCL2ROSMsg ------------------------------- */
                pcl::toROSMsg(point_cloud_,point_cloud_msg_);
                /* -------------------------------------------------------------------------- */

                /* ------------------------- ROS Message - Creation ------------------------- */
                point_cloud_msg_.header.stamp = this->now();
                point_cloud_msg_.header.frame_id = this->reference_frame;
                /* -------------------------------------------------------------------------- */

                /* ------------------------ ROS Message - Publishing ------------------------ */
                this->pc_publisher_->publish(point_cloud_msg_);
                /* -------------------------------------------------------------------------- */    
            }

            void updateReferenceFrame(const std::string& reference_frame_)
            {
                this->reference_frame = reference_frame_;
            }
            /* -------------------------------------------------------------------------- */

        /* -------------------------------------------------------------------------- */
    };
};
/* -------------------------------------------------------------------------- */