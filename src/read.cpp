#include <read.hpp>

#include <stdexcept>
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>

namespace e57_pcl {

cloud_normal_t::Ptr read_scan_with_normals(e57::Reader& reader, uint32_t scan_index);

cloud_normal_t::Ptr load_e57_cloud_with_normals(const std::string& input_file, std::string& guid) {
    cloud_normal_t::Ptr cloud(new cloud_normal_t());
    std::vector<cloud_normal_t::Ptr> scans = load_e57_scans_with_normals(input_file, guid);
    for (auto scan : scans) {
        cloud->insert(cloud->end(), scan->begin(), scan->end());
    }
    return cloud;
}

std::vector<cloud_normal_t::Ptr> load_e57_scans_with_normals(const std::string& input_file, std::string& guid) {
    std::vector<cloud_normal_t::Ptr> scans;

    try {
        e57::Reader reader(input_file);
        e57::E57Root root;
        reader.GetE57Root(root);

        guid = root.guid;

        uint32_t scanCount = reader.GetData3DCount();

		for (uint32_t scan_index = 0; scan_index < scanCount; ++scan_index) {
            scans.push_back(read_scan_with_normals(reader, scan_index));
		}
    } catch(std::exception& e) {
        throw std::runtime_error(std::string("Exception while loading E57 file with normal data:\n") + e.what());
    }

    return scans;
}

cloud_normal_t::Ptr read_scan_with_normals(e57::Reader& reader, uint32_t scan_index) {
    e57::Data3D header;
    reader.ReadData3D(scan_index, header);
    int64_t nColumn = 0, nRow = 0, nPointsSize = 0, nGroupsSize = 0, nCounts = 0; bool bColumnIndex = 0;
    reader.GetData3DSizes( scan_index, nRow, nColumn, nPointsSize, nGroupsSize, nCounts, bColumnIndex);

    int64_t n_size = (nRow > 0) ? nRow : 1024;

    double *data_x = new double[n_size], *data_y = new double[n_size], *data_z = new double[n_size];//, *intensity = new double[n_size];
    double *nrm_x = new double[n_size], *nrm_y = new double[n_size], *nrm_z = new double[n_size];
    int8_t valid_normals = 0;
    auto block_read = reader.SetUpData3DPointsData(
        scan_index,
        n_size,
        data_x,
        data_y,
        data_z,
        NULL,
        NULL, // intesity
        NULL, NULL, NULL, NULL, NULL,
        nrm_x,
        nrm_y,
        nrm_z,
        &valid_normals
    );

    unsigned long size = 0;
    //float imin = header.intensityLimits.intensityMinimum;
    //float imax = header.intensityLimits.intensityMaximum;
    cloud_normal_t::Ptr scan(new cloud_normal_t());
    while((size = block_read.read()) > 0) {
        for(unsigned long i = 0; i < size; i++) {
            point_normal_t p;
            p.x = data_x[i];
            p.y = data_y[i];
            p.z = data_z[i];
            p.normal[0] = nrm_x[i];
            p.normal[1] = nrm_y[i];
            p.normal[2] = nrm_z[i];
            //p.intensity = (intensity[i]-imin) / (imax-imin);
            scan->push_back(p);
        }
    }
    block_read.close();

    delete [] data_x;
    delete [] data_y;
    delete [] data_z;
    delete [] nrm_x;
    delete [] nrm_y;
    delete [] nrm_z;

    // get origin position
    scan->sensor_origin_ = Eigen::Vector4f(header.pose.translation.x, header.pose.translation.y, header.pose.translation.z, 1.0);

    return scan;
}

} // e57_pcl
