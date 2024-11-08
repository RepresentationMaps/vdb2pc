/*********************************************************************
 *
 * Software License Agreement
 * 
 * Copyright (c) 2024, [Anonymous]
 * All rights reserved.
 * 
 * This file is part of VDB2PC, derived from spatio_temporal_voxel_layer.
 * (https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/ros2)
 * Original work Copyright (c) 2018, Simbe Robotics, Inc.
 * 
 * Modifications Copyright (c) 2024, [Anonymous]
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
 * Authors: Anonymous
 * Purpose: convert VDB structures into PointCloud
 * 
 *********************************************************************/

#ifndef VDB2PC_HPP_
#define VDB2PC_HPP_

#include <iostream>
#include <type_traits> 
#include <openvdb/openvdb.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>

namespace vdb2pc
{
    template<class T>
    void transform(T &vdb_grid, pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        static_assert(std::is_base_of_v<openvdb::GridBase, T>);
        static_assert(!std::is_same_v<openvdb::GridBase, T>);
        openvdb::initialize();

        using ValueOnCIter_ = typename T::ValueOnCIter;
        for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
        {
            const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord());

            cloud.push_back(pcl::PointXYZ(worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2]));
        }
    }

    template<class T>
    void transform(T &vdb_grid, pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        static_assert(std::is_base_of_v<openvdb::GridBase, T>);
        static_assert(!std::is_same_v<openvdb::GridBase, T>);
        openvdb::initialize();

        using ValueOnCIter_ = typename T::ValueOnCIter;
        for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter)
        {
            const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord());
            auto intensity = iter.getValue();

            cloud.push_back(pcl::PointXYZI(
                worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2], intensity));
        }
    }
}; // namespace vdb2pc
#endif // VDB2PC_HPP_