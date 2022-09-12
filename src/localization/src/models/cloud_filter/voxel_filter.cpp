/*
 * @Description  : voxel filter model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:51:52
 */

#define PCL_NO_PRECOMPILE
#include "models/cloud_filter/voxel_filter.hpp"

#include <vector>

#include <glog/logging.h>
#include <pcl/common/common.h>

#include "sensor_data/cloud_data.hpp"

namespace localization
{
VoxelFilter::VoxelFilter(const YAML::Node& node)
    : inverse_leaf_size_(Eigen::Array4f::Zero())
{
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();

    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    LOG(INFO) << "Voxel Filter params:";
    LOG(INFO) << leaf_size_x << ", " << leaf_size_y << ", " << leaf_size_z;

    // Set inverse_leaf_size for segmentation use
    Eigen::Vector4f leaf_size(Eigen::Vector4f::Zero());
    leaf_size[0] = leaf_size_x;
    leaf_size[1] = leaf_size_y;
    leaf_size[2] = leaf_size_z;
    leaf_size[3] = 1;
    inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size.array();

    return true;
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr)
{
    // Get min and max point
    Eigen::Vector4f min_p, max_p;
    pcl::getMinMax3D(*input_cloud_ptr, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
    {
        CloudData::CLOUD_PTR cloud_left(new CloudData::CLOUD()), cloud_right(new CloudData::CLOUD());
        CloudData::CLOUD_PTR cloud_left_filter(new CloudData::CLOUD()), cloud_right_filter(new CloudData::CLOUD());
        pcl::PassThrough<CloudData::POINT> passthrough_filter;

        if (dx > dy && dx > dz)
        {
            passthrough_filter.setInputCloud(input_cloud_ptr);
            passthrough_filter.setFilterFieldName("x");
            passthrough_filter.setFilterLimits(min_p[0], min_p[0] + (max_p[0] - min_p[0]) / 2);
            passthrough_filter.filter(*cloud_left);

            passthrough_filter.setFilterLimitsNegative(true);
            passthrough_filter.filter(*cloud_right);
        }
        else if (dy > dx && dy > dz)
        {
            passthrough_filter.setInputCloud(input_cloud_ptr);
            passthrough_filter.setFilterFieldName("y");
            passthrough_filter.setFilterLimits(min_p[1], min_p[1] + (max_p[1] - min_p[1]) / 2);
            passthrough_filter.filter(*cloud_left);

            passthrough_filter.setFilterLimitsNegative(true);
            passthrough_filter.filter(*cloud_right);
        }
        else
        {
            passthrough_filter.setInputCloud(input_cloud_ptr);
            passthrough_filter.setFilterFieldName("z");
            passthrough_filter.setFilterLimits(min_p[2], min_p[2] + (max_p[2] - min_p[2]) / 2);
            passthrough_filter.filter(*cloud_left);

            passthrough_filter.setFilterLimitsNegative(true);
            passthrough_filter.filter(*cloud_right);
        }

        this->Filter(cloud_left, cloud_left_filter);
        this->Filter(cloud_right, cloud_right_filter);
        *filtered_cloud_ptr = *cloud_left_filter + *cloud_right_filter;
    }
    else
    {
        voxel_filter_.setInputCloud(input_cloud_ptr);
        voxel_filter_.filter(*filtered_cloud_ptr);
    }

    return true;
}
}  // namespace localization