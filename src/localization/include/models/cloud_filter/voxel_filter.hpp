/*
 * @Description  : voxel filter model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:50:50
 */
#pragma once
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace localization
{
class VoxelFilter : public CloudFilterInterface
{
  public:
    /**
     * @brief  构造该类需要的参数
     * @param leaf_size_x 每一个体素的长
     * @param leaf_size_y 每一个体素的宽
     * @param leaf_size_z 每一个体素的高
     */
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x,
                float leaf_size_y,
                float leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    /**
     * @brief  设置体素滤波器参数
     * @param leaf_size_x 每一个体素的长
     * @param leaf_size_y 每一个体素的宽
     * @param leaf_size_z 每一个体素的高
     * @return  true
     */
    bool SetFilterParam(float leaf_size_x,
                        float leaf_size_y,
                        float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;

    // Param
    Eigen::Array4f inverse_leaf_size_;
};
}  // namespace localization