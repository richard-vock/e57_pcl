#include <read.hpp>

#include <stdexcept>
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>

namespace e57_pcl {

typedef std::vector<double> coords_t;
typedef std::vector<uint16_t> colors_t;
typedef std::tuple<coords_t, coords_t, coords_t> point_data_t;
typedef std::tuple<colors_t, colors_t, colors_t> color_data_t;

uint32_t
get_scan_count(const std::string& input_file) {
    uint32_t scan_count;
    try {
        e57::Reader reader(input_file);
        e57::E57Root root;
        reader.GetE57Root(root);
        scan_count = reader.GetData3DCount();
    } catch (std::exception& e) {
        throw std::runtime_error(
            std::string(
                "Exception while loading E57 file with normal data:\n") +
            e.what());
    }

    return scan_count;
}

cloud_normal_t::Ptr read_scan_with_normals(
    e57::Reader& reader, uint32_t scan_index,
    const Eigen::Vector3d& demean_offset,
    std::vector<Eigen::Vector3f>* colors = nullptr);

cloud_normal_t::Ptr
load_e57_cloud_with_normals(const std::string& input_file, std::string& guid,
                            bool demean, Eigen::Vector3d* demean_offset, std::vector<Eigen::Vector3f>* colors) {
    cloud_normal_t::Ptr cloud(new cloud_normal_t());
    std::vector<cloud_normal_t::Ptr> scans =
        load_e57_scans_with_normals(input_file, guid, demean, demean_offset, std::vector<uint32_t>(), colors);
    for (auto scan : scans) {
        cloud->insert(cloud->end(), scan->begin(), scan->end());
    }
    return cloud;
}

std::vector<cloud_normal_t::Ptr>
load_e57_scans_with_normals(const std::string& input_file, std::string& guid,
                            bool demean, Eigen::Vector3d* demean_offset,
                            const std::vector<uint32_t>& scans_indices,
                            std::vector<Eigen::Vector3f>* colors) {
    std::vector<cloud_normal_t::Ptr> scans;

    try {
        e57::Reader reader(input_file);
        e57::E57Root root;
        reader.GetE57Root(root);

        guid = root.guid;

        uint32_t scan_count = reader.GetData3DCount();
        std::vector<uint32_t> scan_subset;
        if (scans_indices.empty()) {
            scan_subset = std::vector<uint32_t>(scan_count);
            std::iota(scan_subset.begin(), scan_subset.end(), 0);
        } else {
            scan_subset = scans_indices;
        }

        Eigen::Vector3d tmp_demean_offset = Eigen::Vector3d::Zero();
        if (demean) {
            e57::Data3D first_scan_header;
            reader.ReadData3D(0, first_scan_header);
            tmp_demean_offset[0] = -first_scan_header.pose.translation.x;
            tmp_demean_offset[1] = -first_scan_header.pose.translation.y;
            tmp_demean_offset[2] = -first_scan_header.pose.translation.z;
        }

        if (colors) colors->clear();
        for (uint32_t scan_index : scan_subset) {
            scans.push_back(
                read_scan_with_normals(reader, scan_index, tmp_demean_offset, colors));
        }

        if (demean && demean_offset) {
            (*demean_offset) = tmp_demean_offset;
        }
    } catch (std::exception& e) {
        throw std::runtime_error(
            std::string(
                "Exception while loading E57 file with normal data:\n") +
            e.what());
    }

    return scans;
}

cloud_normal_t::Ptr
read_scan_with_normals(e57::Reader& reader, uint32_t scan_index,
                       const Eigen::Vector3d& demean_offset,
                       std::vector<Eigen::Vector3f>* colors) {
    e57::Data3D header;
    reader.ReadData3D(scan_index, header);
    int64_t nColumn = 0, nRow = 0, nPointsSize = 0, nGroupsSize = 0,
            nCounts = 0;
    bool bColumnIndex = 0;
    reader.GetData3DSizes(scan_index, nRow, nColumn, nPointsSize, nGroupsSize,
                          nCounts, bColumnIndex);

    int64_t n_size = (nRow > 0) ? nRow : 1024;

    double* data_x = new double[n_size], * data_y = new double[n_size],
            * data_z = new double[n_size];
    double* nrm_x = new double[n_size], * nrm_y = new double[n_size],
            * nrm_z = new double[n_size];
    uint16_t* col_r = colors ? new uint16_t[n_size] : NULL;
    uint16_t* col_g = colors ? new uint16_t[n_size] : NULL;
    uint16_t* col_b = colors ? new uint16_t[n_size] : NULL;
    int8_t valid_normals = 0, valid_colors = 0;
    auto block_read = reader.SetUpData3DPointsData(
        scan_index, n_size, data_x, data_y, data_z, NULL, NULL, NULL,
        col_r, col_g, col_b, &valid_colors, nrm_x, nrm_y, nrm_z, &valid_normals);

    Eigen::Affine3d rotation;
    rotation =
        Eigen::Quaterniond(header.pose.rotation.w, header.pose.rotation.x,
                           header.pose.rotation.y, header.pose.rotation.z);
    Eigen::Affine3d registration;
    registration = Eigen::Translation<double, 3>(
                       header.pose.translation.x + demean_offset[0],
                       header.pose.translation.y + demean_offset[1],
                       header.pose.translation.z + demean_offset[2]) *
                   rotation;
    Eigen::Affine3d normal_transformation;
    normal_transformation = rotation.matrix().inverse().transpose();

    unsigned long size = 0;
    cloud_normal_t::Ptr scan(new cloud_normal_t());
    while ((size = block_read.read()) > 0) {
        for (unsigned long i = 0; i < size; i++) {
            point_normal_t p;
            Eigen::Vector3d pos(data_x[i], data_y[i], data_z[i]);
            pos = registration * pos;
            p.x = static_cast<float>(pos[0]);
            p.y = static_cast<float>(pos[1]);
            p.z = static_cast<float>(pos[2]);
            Eigen::Vector3d nrm(nrm_x[i], nrm_y[i], nrm_z[i]);
            nrm = normal_transformation * nrm;
            p.normal[0] = static_cast<float>(nrm[0]);
            p.normal[1] = static_cast<float>(nrm[1]);
            p.normal[2] = static_cast<float>(nrm[2]);
            scan->push_back(p);

            if (colors) {
                Eigen::Vector3f p_col;
                float max_col = 255.f;
                p_col <<
                    static_cast<float>(col_r[i]) / max_col,
                    static_cast<float>(col_g[i]) / max_col,
                    static_cast<float>(col_b[i]) / max_col;
                colors->push_back(p_col);
            }
        }
    }
    block_read.close();

    delete[] data_x;
    delete[] data_y;
    delete[] data_z;
    delete[] nrm_x;
    delete[] nrm_y;
    delete[] nrm_z;
    if (colors) {
        delete[] col_r;
        delete[] col_g;
        delete[] col_b;
    }

    // get origin position
    scan->sensor_origin_ =
        Eigen::Vector4f(header.pose.translation.x + demean_offset[0],
                        header.pose.translation.y + demean_offset[1],
                        header.pose.translation.z + demean_offset[2], 1.0);

    return scan;
}

}  // e57_pcl
