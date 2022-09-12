/*
 * @Description  : no filter model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:50:40
 */

#pragma once

#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace localization
{
class NoFilter : public CloudFilterInterface
{
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}  // namespace localization
