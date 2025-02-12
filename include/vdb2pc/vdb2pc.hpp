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
 * Purpose: convert VDB structures into PCL pointclouds (and viceversa)
 *
 *********************************************************************/

#ifndef VDB2PC__VDB2PC_HPP_
#define VDB2PC__VDB2PC_HPP_

#include <openvdb/openvdb.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <iostream>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

namespace vdb2pc
{
template<class T>
void transform(T & vdb_grid, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  static_assert(std::is_base_of_v<openvdb::GridBase, T>);
  static_assert(!std::is_same_v<openvdb::GridBase, T>);
  openvdb::initialize();

  using ValueOnCIter_ = typename T::ValueOnCIter;
  for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter) {
    const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord());

    cloud.push_back(pcl::PointXYZ(worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2]));
  }
}

template<class T>
void transform(T & vdb_grid, pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  static_assert(std::is_base_of_v<openvdb::GridBase, T>);
  static_assert(!std::is_same_v<openvdb::GridBase, T>);
  openvdb::initialize();

  using ValueOnCIter_ = typename T::ValueOnCIter;
  for (ValueOnCIter_ iter = vdb_grid.cbeginValueOn(); iter; ++iter) {
    const openvdb::Vec3d worldSpacePoint = vdb_grid.indexToWorld(iter.getCoord());
    auto intensity = iter.getValue();

    cloud.push_back(
      pcl::PointXYZI(
        worldSpacePoint[0], worldSpacePoint[1], worldSpacePoint[2], intensity));
  }
}
}  // namespace vdb2pc
#endif  // VDB2PC__VDB2PC_HPP_
