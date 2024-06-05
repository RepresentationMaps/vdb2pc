/* -------------------------------------------------------------------------- */
/*                                Header Files                                */
/* -------------------------------------------------------------------------- */

/* ----------------------------------- STL ---------------------------------- */
#include <iostream>
/* -------------------------------------------------------------------------- */

/* ------------------------------- Type Traits ------------------------------ */
#include <type_traits> 
/* -------------------------------------------------------------------------- */

/* --------------------------------- OpenVDB -------------------------------- */
#include <openvdb/openvdb.h>
/* -------------------------------------------------------------------------- */

/* ---------------------------- PCL - Point Types --------------------------- */
#include <pcl/point_types.h> // For basic point cloud types (e.g., pcl::PointXYZ, pcl::PointXYZI)
/* -------------------------------------------------------------------------- */

/* ------------------------------- Point Cloud ------------------------------ */
#include <pcl/point_cloud.h> // For point cloud data structures (e.g., pcl::PointCloud)
/* -------------------------------------------------------------------------- */

/* --------------------------- ROS Client Library --------------------------- */
#include <rclcpp/rclcpp.hpp>
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                  Namespace                                 */
/* -------------------------------------------------------------------------- */
namespace vdb_utilities{
    /* -------------------------------------------------------------------------- */
    /*                               Class Template                               */
    /* -------------------------------------------------------------------------- */
    template<class T>
    class VDB2PointCloud
    {
        /* --------------------------------- Private -------------------------------- */
        private:

        /* -------------------------------------------------------------------------- */

        /* --------------------------------- Public --------------------------------- */
        public:
            /* ------------------------------- Constructor ------------------------------ */
            VDB2PointCloud()
            {
                /* ---------------------------------- Check --------------------------------- */
                static_assert(std::is_base_of_v<openvdb::GridBase,T>); // Ensures that the type T is derived from the openvdb::GridBase class.
                static_assert(!std::is_same_v<openvdb::GridBase,T>); // Ensures that the type T is not exactly the same as the openvdb::GridBase class.
                /* -------------------------------------------------------------------------- */

                /* ----------------------------- Initialization ----------------------------- */
                openvdb::initialize();
                /* -------------------------------------------------------------------------- */
            }
            /* -------------------------------------------------------------------------- */
            
            /* ------------------------------- Destructor ------------------------------- */
            ~VDB2PointCloud()
            {

            }
            /* -------------------------------------------------------------------------- */

            /* -------------------------------- Transform ------------------------------- */
            void transform(T& vdb_grid, pcl::PointCloud<pcl::PointXYZ>& cloud)
            {
                /**------------------------------------------------------------------------
                 **                            INFO HEADER

                    - The keyword typename was introduced to specify that the identifier 
                    that follows is a type. 

                    - Iterator classes follow a fairly consistent naming scheme. First, 
                    the CIter and Iter suffixes denote const and non-const iterators, 
                    i.e., iterators that offer, respectively, read-only and read/write 
                    access to the underlying tree or node. 

                    - Second, iterators over tile and voxel values are denoted either On, 
                    Off or All, indicating that they visit only active values, only 
                    inactive values, or both active and inactive values.

                    - Grid::cbeginValueOn returns a const iterator to the first of a 
                    grid’s active values

                    - Translation from index coordinates (i, j,  k) to world space 
                    coordinates (x, y, z) is done with a call to the indexToWorld 
                    method, and from world space coordinates to index space coordinates with 
                    a call to worldToIndex 

                        - openvdb::Vec3d worldSpacePoint = linearTransform->indexToWorld(ijk);
                        - openvdb::Vec3d indexSpacePoint = linearTransform->worldToIndex(worldSpacePoint);
                *------------------------------------------------------------------------**/
                
                /* -------------------------------- Aliasing -------------------------------- */
                using ValueOnCIter_ = typename T::ValueOnCIter;
                /* -------------------------------------------------------------------------- */

                /* ----------------------------- Transformation ----------------------------- */
                for(ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
                {
                    /* ----------------------- Active Vaules' Coordinates ----------------------- */
                    const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord()); // Compute the location in world space that is the image of `iter.getCoord()`.
                    /* -------------------------------------------------------------------------- */

                    /* -------------------------------- Push Back ------------------------------- */
                    cloud.push_back(pcl::PointXYZ(worldSpacePoint[0],worldSpacePoint[1],worldSpacePoint[2]));
                    /* -------------------------------------------------------------------------- */
                }
                /* -------------------------------------------------------------------------- */
                    
            }
            void transform(T& vdb_grid, pcl::PointCloud<pcl::PointXYZI>& cloud)
            {  
                /**------------------------------------------------------------------------
                 **                            INFO HEADER

                    - The keyword typename was introduced to specify that the identifier 
                    that follows is a type. 

                    - Iterator classes follow a fairly consistent naming scheme. First, 
                    the CIter and Iter suffixes denote const and non-const iterators, 
                    i.e., iterators that offer, respectively, read-only and read/write 
                    access to the underlying tree or node. 

                    - Second, iterators over tile and voxel values are denoted either On, 
                    Off or All, indicating that they visit only active values, only 
                    inactive values, or both active and inactive values.

                    - Grid::cbeginValueOn returns a const iterator to the first of a 
                    grid’s active values

                    - Translation from index coordinates (i, j,  k) to world space 
                    coordinates (x, y, z) is done with a call to the indexToWorld 
                    method, and from world space coordinates to index space coordinates with 
                    a call to worldToIndex 

                        - openvdb::Vec3d worldSpacePoint = linearTransform->indexToWorld(ijk);
                        - openvdb::Vec3d indexSpacePoint = linearTransform->worldToIndex(worldSpacePoint);
                *------------------------------------------------------------------------**/
                
                /* -------------------------------- Aliasing -------------------------------- */
                using ValueOnCIter_ = typename T::ValueOnCIter;
                /* -------------------------------------------------------------------------- */

                /* ----------------------------- Transformation ----------------------------- */
                for(ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
                {
                    /* ----------------------- Active Voxels' Coordinates ----------------------- */
                    const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord()); // Compute the location in world space that is the image of `iter.getCoord()`.
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------ Voxels Value ------------------------------ */
                    auto intensity = iter.getValue();
                    /* -------------------------------------------------------------------------- */

                    /* -------------------------------- Push Back ------------------------------- */
                    cloud.push_back(pcl::PointXYZI(worldSpacePoint[0],worldSpacePoint[1],worldSpacePoint[2],intensity));
                    /* -------------------------------------------------------------------------- */
                }
                /* -------------------------------------------------------------------------- */
            }
            /* -------------------------------------------------------------------------- */

        /* -------------------------------------------------------------------------- */
    };
    /* -------------------------------------------------------------------------- */
};
/* -------------------------------------------------------------------------- */