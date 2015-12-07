#include <write.hpp>

#include <stdexcept>
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>

namespace e57_pcl {

typedef std::vector<double> coords_t;
typedef std::vector<uint16_t> colors_t;
typedef std::tuple<coords_t, coords_t, coords_t> point_data_t;
typedef std::tuple<colors_t, colors_t, colors_t> color_data_t;

void write_e57n(const std::string& output_file, cloud_normal_t::ConstPtr cloud, const std::string& guid,
               std::vector<Eigen::Vector3f>* colors) {
    e57::Writer writer(output_file, "Not in use.");
    e57::Data3D header;
    header.guid = guid.c_str();
    uint32_t num_points = cloud->size();
    header.pointsSize = num_points * 3;
    header.pointFields.cartesianXField = true;
    header.pointFields.cartesianYField = true;
    header.pointFields.cartesianZField = true;
    coords_t dummy(num_points, 0.0);
    point_data_t pos_data(dummy, dummy, dummy);
    point_data_t nrm_data(dummy, dummy, dummy);
    uint16_t max_col = 255;
    color_data_t col_data(colors_t(num_points, max_col), colors_t(num_points, max_col), colors_t(num_points, max_col));
    if (colors) {
        for (uint32_t i = 0; i < num_points; ++i) {
            std::get<0>(col_data)[i] = static_cast<uint16_t>((*colors)[i][0] * 255.f);
            std::get<1>(col_data)[i] = static_cast<uint16_t>((*colors)[i][1] * 255.f);
            std::get<2>(col_data)[i] = static_cast<uint16_t>((*colors)[i][2] * 255.f);
        }
    }
    uint32_t idx = 0;
    for (const auto& p : *cloud) {
        std::get<0>(pos_data)[idx] = p.x;
        std::get<1>(pos_data)[idx] = p.y;
        std::get<2>(pos_data)[idx] = p.z;
        std::get<0>(nrm_data)[idx] = p.normal[0];
        std::get<1>(nrm_data)[idx] = p.normal[1];
        std::get<2>(nrm_data)[idx] = p.normal[2];
        ++idx;
    }

    header.pointFields.cartesianXField = true;
    header.pointFields.cartesianYField = true;
    header.pointFields.cartesianZField = true;
    header.pointFields.sphericalRangeField = true;
    header.pointFields.sphericalAzimuthField = true;
    header.pointFields.sphericalElevationField = true;
    header.pointFields.colorBlueField = true;
    header.pointFields.colorGreenField = true;
    header.pointFields.colorRedField = true;
    header.colorLimits.colorRedMinimum = 0;
    header.colorLimits.colorRedMaximum = 255;
    header.colorLimits.colorGreenMinimum = 0;
    header.colorLimits.colorGreenMaximum = 255;
    header.colorLimits.colorBlueMinimum = 0;
    header.colorLimits.colorBlueMaximum = 255;

    uint16_t max_c = 0;
    for (uint32_t i = 0; i < num_points; ++i) {
        uint16_t c = std::get<0>(col_data)[i];
        if (c > max_c) max_c = c;
        c = std::get<1>(col_data)[i];
        if (c > max_c) max_c = c;
        c = std::get<2>(col_data)[i];
        if (c > max_c) max_c = c;
    }

    int scan_index = writer.NewData3D(header);

    int8_t pos_valid = 1, nrm_valid = 1, col_valid = 1;
    e57::CompressedVectorWriter block_write = writer.SetUpData3DPointsData(
        scan_index,
        num_points,
        std::get<0>(pos_data).data(),
        std::get<1>(pos_data).data(),
        std::get<2>(pos_data).data(),
        &pos_valid,
        NULL, NULL,
        std::get<0>(col_data).data(),
        std::get<1>(col_data).data(),
        std::get<2>(col_data).data(),
        &col_valid,
        std::get<0>(nrm_data).data(),
        std::get<1>(nrm_data).data(),
        std::get<2>(nrm_data).data(),
        &nrm_valid
    );
    block_write.write(num_points);
    block_write.close();
}

} // e57_pcl
