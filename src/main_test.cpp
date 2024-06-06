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
#include <string>
#include <openvdb/tools/LevelSetSphere.h>
#include "vdb2pc_pub.hpp"

std::shared_ptr<rclcpp::Node> node_ = nullptr;
openvdb::FloatGrid::Ptr float_grid;
openvdb::Int32Grid::Ptr int_grid;
openvdb::BoolGrid::Ptr bool_grid;
std::shared_ptr<ros_vdb2pc::VDB2PCPublisher<openvdb::FloatGrid>> float_vdb_publisher_;
std::shared_ptr<ros_vdb2pc::VDB2PCPublisher<openvdb::Int32Grid>> int_vdb_publisher_;
std::shared_ptr<ros_vdb2pc::VDB2PCPublisher<openvdb::BoolGrid>> bool_vdb_publisher_;

void timerCallback()
{
    RCLCPP_INFO(node_->get_logger(),"Callback");
    float_vdb_publisher_->publish(*float_grid);
    int_vdb_publisher_->publish(*int_grid);
    bool_vdb_publisher_->publish(*bool_grid);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    node_ = std::make_shared<rclcpp::Node>("TestNode");
    auto timer = node_->create_wall_timer(std::chrono::milliseconds(200), timerCallback);

    openvdb::initialize();

    
    openvdb::Vec3f float_center(0.0,0.0,0.0);


    float_grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0,float_center,0.1);
    
    int_grid = openvdb::Int32Grid::create(0);
    openvdb::Int32Grid::Accessor int_grid_accessor = int_grid->getAccessor();

    bool_grid = openvdb::BoolGrid::create(0);
    openvdb::BoolGrid::Accessor bool_grid_accessor = bool_grid->getAccessor();
    
    for(int x = 0; x < 3; ++x)
        for(int y = 0; y < 3; ++y)
            for(int z = 2; z < 5; ++z)
            {
                /* -------------------------------- Int Grid -------------------------------- */
                int_grid_accessor.setValue(openvdb::Coord(x,y-3,z),z);
                /* -------------------------------------------------------------------------- */

                /* -------------------------------- Bool Grid ------------------------------- */
                bool_grid_accessor.setValue(openvdb::Coord(x,y+3,z+2),1);
                /* -------------------------------------------------------------------------- */
            }

    float_vdb_publisher_ = std::make_shared<ros_vdb2pc::VDB2PCPublisher<openvdb::FloatGrid>>(std::string("float_test_node").c_str(),std::string("/test_float_topic").c_str(),std::string("map").c_str());

    int_vdb_publisher_ = std::make_shared<ros_vdb2pc::VDB2PCPublisher<openvdb::Int32Grid>>(std::string("int_test_node").c_str(),std::string("/test_int_topic").c_str(),std::string("map").c_str());

    bool_vdb_publisher_ = std::make_shared<ros_vdb2pc::VDB2PCPublisher<openvdb::BoolGrid>>(std::string("bool_test_node").c_str(),std::string("/test_bool_topic").c_str(),std::string("map").c_str());

    rclcpp::spin(node_);
    rclcpp::shutdown();

    return 0;
}