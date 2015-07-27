#ifndef _E57_PCL_READ_HPP_
#define _E57_PCL_READ_HPP_

#include "types.hpp"

namespace e57_pcl {

cloud_normal_t::Ptr
load_e57_cloud_with_normals(const std::string& input_file, std::string& guid);

std::vector<cloud_normal_t::Ptr>
load_e57_scans_with_normals(const std::string& input_file, std::string& guid);

} // e57_pcl

#endif /* _E57_PCL_READ_HPP_ */
