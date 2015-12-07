#ifndef _E57_PCL_WRITE_HPP_
#define _E57_PCL_WRITE_HPP_

#include "types.hpp"
#include <Eigen/Dense>

namespace e57_pcl {

void write_e57n(const std::string& output_file, cloud_normal_t::ConstPtr cloud,
                const std::string& guid,
                std::vector<Eigen::Vector3f>* colors = nullptr);

} // e57_pcl

#endif /* _E57_PCL_WRITE_HPP_ */
