#ifndef _E57_PCL_TYPES_HPP_
#define _E57_PCL_TYPES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace e57_pcl {

typedef pcl::PointXYZ point_xyz_t;
typedef pcl::PointNormal point_normal_t;

typedef pcl::PointCloud<point_xyz_t> cloud_xyz_t;
typedef pcl::PointCloud<point_normal_t> cloud_normal_t;

} // e57_pcl

#endif /* _E57_PCL_TYPES_HPP_ */
