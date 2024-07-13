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

#ifndef VDB2PC_HPP_
#define VDB2PC_HPP_

#include <iostream>
#include <type_traits> 
#include <openvdb/openvdb.h>
#include <pcl/point_types.h> // For basic point cloud types (e.g., pcl::PointXYZ, pcl::PointXYZI)
#include <pcl/point_cloud.h> // For point cloud data structures (e.g., pcl::PointCloud)
#include <rclcpp/rclcpp.hpp>

namespace vdb2pc
{
    template<class T>
    void transform(T &vdb_grid, pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        static_assert(std::is_base_of_v<openvdb::GridBase, T>); // Ensures that the type T is derived from the openvdb::GridBase class.
        static_assert(!std::is_same_v<openvdb::GridBase, T>);   // Ensures that the type T is not exactly the same as the openvdb::GridBase class.
        openvdb::initialize();

        using ValueOnCIter_ = typename T::ValueOnCIter;
        for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
        {
            const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord()); // Compute the location in world space that is the image of `iter.getCoord()`.

            cloud.push_back(pcl::PointXYZ(worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2]));
        }
    }

    template<class T>
    void transform(T &vdb_grid, pcl::PointCloud<pcl::PointXYZI> &cloud)
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

        static_assert(std::is_base_of_v<openvdb::GridBase, T>); // Ensures that the type T is derived from the openvdb::GridBase class.
        static_assert(!std::is_same_v<openvdb::GridBase, T>);   // Ensures that the type T is not exactly the same as the openvdb::GridBase class.
        openvdb::initialize();

        using ValueOnCIter_ = typename T::ValueOnCIter;
        for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
        {
            const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord()); // Compute the location in world space that is the image of `iter.getCoord()`.
            auto intensity = iter.getValue();

            cloud.push_back(pcl::PointXYZI(worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2], intensity));
        }
    }
}; // namespace vdb2pc
#endif // VDB2PC_HPP_