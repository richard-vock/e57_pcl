#ifndef _E57_PCL_READ_HPP_
#define _E57_PCL_READ_HPP_

#include "types.hpp"
#include <Eigen/Dense>

namespace e57_pcl {

uint32_t
get_scan_count(const std::string& input_file);

Eigen::Vector3d
get_first_scan_origin(const std::string& input_file, const std::vector<uint32_t>& scans_indices = std::vector<uint32_t>());

cloud_normal_t::Ptr
load_e57_cloud_with_normals(const std::string& input_file, std::string& guid,
                            bool demean = false,
                            Eigen::Vector3d* demean_offset = nullptr);

std::vector<cloud_normal_t::Ptr>
load_e57_scans_with_normals(const std::string& input_file, std::string& guid,
                            bool demean = false,
                            Eigen::Vector3d* demean_offset = nullptr,
                            const std::vector<uint32_t>& scan_indices = std::vector<uint32_t>());

} // e57_pcl

#endif /* _E57_PCL_READ_HPP_ */
