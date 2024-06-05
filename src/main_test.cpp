/* -------------------------------------------------------------------------- */
/*                                Header Files                                */
/* -------------------------------------------------------------------------- */

/* --------------------------------- String --------------------------------- */
#include <string>
/* -------------------------------------------------------------------------- */

/* --------------------------------- OpenVDB -------------------------------- */
#include <openvdb/tools/LevelSetSphere.h>
/* -------------------------------------------------------------------------- */

/* ----------------------------- VDB2PointCloud ----------------------------- */
#include "vdb2pc_pub.hpp"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                               Global Elements                              */
/* -------------------------------------------------------------------------- */
std::shared_ptr<rclcpp::Node> node_ = nullptr;
openvdb::FloatGrid::Ptr float_grid;
openvdb::Int32Grid::Ptr int_grid;
openvdb::BoolGrid::Ptr bool_grid;
std::shared_ptr<ros_vdb_utilities::VDB2PCPublisher<openvdb::FloatGrid>> float_vdb_publisher_;
std::shared_ptr<ros_vdb_utilities::VDB2PCPublisher<openvdb::Int32Grid>> int_vdb_publisher_;
std::shared_ptr<ros_vdb_utilities::VDB2PCPublisher<openvdb::BoolGrid>> bool_vdb_publisher_;
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */
void timerCallback()
{
    RCLCPP_INFO(node_->get_logger(),"Callback");
    float_vdb_publisher_->publish(*float_grid);
    int_vdb_publisher_->publish(*int_grid);
    bool_vdb_publisher_->publish(*bool_grid);
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv)
{
    /* ----------------------------- Initialization ----------------------------- */
    rclcpp::init(argc,argv);
    node_ = std::make_shared<rclcpp::Node>("TestNode");
    auto timer = node_->create_wall_timer(std::chrono::milliseconds(200), timerCallback);
    /* -------------------------------------------------------------------------- */

    /* ------------------------------- Processing ------------------------------- */
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

    float_vdb_publisher_ = std::make_shared<ros_vdb_utilities::VDB2PCPublisher<openvdb::FloatGrid>>(std::string("float_test_node").c_str(),std::string("/test_float_topic").c_str(),std::string("map").c_str());

    int_vdb_publisher_ = std::make_shared<ros_vdb_utilities::VDB2PCPublisher<openvdb::Int32Grid>>(std::string("int_test_node").c_str(),std::string("/test_int_topic").c_str(),std::string("map").c_str());

    bool_vdb_publisher_ = std::make_shared<ros_vdb_utilities::VDB2PCPublisher<openvdb::BoolGrid>>(std::string("bool_test_node").c_str(),std::string("/test_bool_topic").c_str(),std::string("map").c_str());

    rclcpp::spin(node_);
    /* -------------------------------------------------------------------------- */

    /* -------------------------------- Shutdown -------------------------------- */
    rclcpp::shutdown();
    /* -------------------------------------------------------------------------- */

    return 0;
}
/* -------------------------------------------------------------------------- */