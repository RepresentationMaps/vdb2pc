/* -------------------------------------------------------------------------- */
/*                                Header Files                                */
/* -------------------------------------------------------------------------- */

/* ----------------------------- VDB2PointCloud ----------------------------- */
#include "vdb2pc.hpp"
/* -------------------------------------------------------------------------- */

/* --------------------------------- OpenVDB -------------------------------- */
#include "openvdb/io/Stream.h"
#include "openvdb/tools/LevelSetSphere.h"
/* -------------------------------------------------------------------------- */

/* ----------------------------------- STL ---------------------------------- */
#include <fstream>
/* -------------------------------------------------------------------------- */

/* ---------------------------------- MSGS ---------------------------------- */
#include "sensor_msgs/msg/point_cloud2.hpp"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

std::shared_ptr<rclcpp::Node> node = nullptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = nullptr;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ = nullptr;

void timerCallback()
{
    RCLCPP_INFO(node->get_logger(), "Hello from ROS2");
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud->height = 1;
    cloud->width = cloud->size();
    pcl::toROSMsg(*cloud,cloud_msg);
    cloud_msg.header.stamp = node->now();
    cloud_msg.header.frame_id = "map";
    pub_->publish(cloud_msg);
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv)
{
    /* ----------------------------------- ROS ---------------------------------- */
    rclcpp::init(argc,argv);
    node = std::make_shared<rclcpp::Node>("my_node_name");
    auto timer = node->create_wall_timer(std::chrono::milliseconds(200), timerCallback);
    pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/test",10);
    /* -------------------------------------------------------------------------- */

    /* ------------------------------- Processing ------------------------------- */
    openvdb::initialize();

    // openvdb::GridPtrVecPtr grids(new openvdb::GridPtrVec);
    // std::ifstream ifile;
    // ifile.open("/root/ros2_ws/src/vdb2pc/armadillo.vdb", std::ios_base::binary);
    // grids = openvdb::io::Stream(ifile).getGrids();
    // ifile.close();

    // openvdb::FloatGrid::Ptr grid;
    // grid = openvdb::gridPtrCast<openvdb::FloatGrid>(grids->at(0)->copyGrid());

    vdb_utilities::VDB2PointCloud<openvdb::FloatGrid> test_obj;
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    

    openvdb::FloatGrid::Ptr grid;
    openvdb::Vec3f center(0.0,0.0,0.0);
    grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0,center,0.1);

    test_obj.transform(*grid,cloud);

    rclcpp::spin(node);
    rclcpp::shutdown();

    // Print all active ("on") voxels by means of an iterator.
    // for(openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
    //     std::cout << "Grid" << iter.getCoord() << " = " << *iter << std::endl;
    /* -------------------------------------------------------------------------- */


    return 0;
}
/* -------------------------------------------------------------------------- */