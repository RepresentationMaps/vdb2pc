/*********************************************************************
 *
 * Software License Agreement
 * 
 * Copyright (c) 2024, [Your Name/Organization]
 * All rights reserved.
 * 
 * This file is part of VDB2PC, derived from [Original Project Name].
 * Original work Copyright (c) 2018, Simbe Robotics, Inc.
 * 
 * Modifications Copyright (c) 2024, [Your Name/Organization]
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
 * Authors: Name (email), Name (email)
 * Purpose: convert VDB structures into PointCloud
 * 
 *********************************************************************/

#include <string>
#include <openvdb/tools/LevelSetSphere.h>
#include "vdb2pc_publisher.hpp"

template<class T, class U>
void timerCallback(const std::shared_ptr<rclcpp::Node>& node, const T& grid, const U& publisher, const std::string& message = "Grid published")
{
    RCLCPP_INFO(node->get_logger(),message.c_str());
    publisher->publish(*grid);
}

int main(int argc, char** argv)
{

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("TestNode");

    openvdb::initialize();

    openvdb::FloatGrid::Ptr float_grid;
    openvdb::Vec3f float_center(0.0,0.0,0.0);
    float_grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0,float_center,0.1);

    openvdb::Int32Grid::Ptr int_grid;
    int_grid = openvdb::Int32Grid::create(0);
    openvdb::Int32Grid::Accessor int_grid_accessor = int_grid->getAccessor();

    openvdb::BoolGrid::Ptr bool_grid;
    bool_grid = openvdb::BoolGrid::create(0);
    openvdb::BoolGrid::Accessor bool_grid_accessor = bool_grid->getAccessor();

    for(int x = 0; x < 3; ++x)
        for(int y = 0; y < 3; ++y)
            for(int z = 2; z < 5; ++z)
            {
                int_grid_accessor.setValue(openvdb::Coord(x,y-3,z),z);
                bool_grid_accessor.setValue(openvdb::Coord(x,y+3,z+2),1);
            }


    std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::FloatGrid>> float_vdb_publisher = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::FloatGrid>>(node,std::string("/test_float_topic").c_str(),std::string("map").c_str());

    std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> int_vdb_publisher = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>>(node,std::string("/test_int_topic").c_str(),std::string("map").c_str());

    std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::BoolGrid>> bool_vdb_publisher = std::make_shared<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::BoolGrid>>(node,std::string("/test_bool_topic").c_str(),std::string("map").c_str());

    auto float_grid_timer = node->create_wall_timer(std::chrono::milliseconds(200),[&node,&float_grid,&float_vdb_publisher](){timerCallback(node,float_grid,float_vdb_publisher,std::string("Float Grid Published!"));});

    auto int_grid_timer = node->create_wall_timer(std::chrono::milliseconds(200),[&node,&int_grid,&int_vdb_publisher](){timerCallback(node,int_grid,int_vdb_publisher,std::string("Int Grid Published!"));});

    auto bool_grid_timer = node->create_wall_timer(std::chrono::milliseconds(200),[&node,&bool_grid,&bool_vdb_publisher](){timerCallback(node,bool_grid,bool_vdb_publisher,std::string("Bool Grid Published!"));});

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}