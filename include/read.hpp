#ifndef _E57_PCL_READ_HPP_
#define _E57_PCL_READ_HPP_

#include "types.hpp"
#include <Eigen/Dense>

namespace e57_pcl {

cloud_normal_t::Ptr
load_e57_cloud_with_normals(const std::string& input_file, std::string& guid, bool demean = false, Eigen::Vector3d* demean_offset = nullptr);

std::vector<cloud_normal_t::Ptr>
load_e57_scans_with_normals(const std::string& input_file, std::string& guid, bool demean = false, Eigen::Vector3d* demean_offset = nullptr);

} // e57_pcl

#endif /* _E57_PCL_READ_HPP_ */
