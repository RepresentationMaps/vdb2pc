vdb2pc
======

A package providing the tools to transform OpenVDB grids into PCL point clouds.
It also provides a ROS-based utility to publish OpenVDB grids as
`sensors_msgs/msg/PointCloud2`, which can be used for visualization
and debugging purposes.

Building
--------

Place the package in the `remap_ws/src`; then, from `remap_ws`

`$ rosdep init`
`$ rosdep update`
`$ rosdep install --from-paths src/vdb2pc`
`$ colcon build --packages-select vdb2pc`

Content
-------

- `include/vdb2pc/vdb2pc.hpp`: tools for transforming OpenVDB grids into point clouds.
- `include/vdb2pc/vdb2pc_publisher.hpp`: here the VDB2PCPublisher class is defined, that is,
	a wrapper around a ROS 2 `sensor_msgs/msg/PointCloud2` Publisher to make it transparent
	to publish an OpenVDB grid.