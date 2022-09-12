/*
 * @Description  : box filter: cut box from cloud points
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:50:18
 */

#pragma once

#include <common/common.h>
#include <pcl/filters/crop_box.h>

#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace localization
{
class BoxFilter : public CloudFilterInterface
{
  public:
    /**
     * @brief  构造该类需要的参数
     * @param size_ box的边界
     */
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    void FilterSet(std::vector<float>& input, int function_type) override;

  private:
    /**
     * @brief  设置box的大小
     * @param  size box的大小
     * @return void
     */
    void SetSize(std::vector<float> size);

    /**
     * @brief  设置box的中心点
     * @param  origin box的中心点
     * @return void
     */
    void SetOrigin(std::vector<float> origin);

    /**
     * @brief  获得box的边界
     * @param  void
     * @return box的边界
     */
    std::vector<float> GetEdge();

    /**
     * @brief  根据box的原点和size计算box的边界
     * @param  void
     * @return void
     */
    void CalculateEdge();

  private:
    pcl::CropBox<CloudData::POINT> pcl_box_filter_;

    // Param
    // Box size
    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}  // namespace localization
