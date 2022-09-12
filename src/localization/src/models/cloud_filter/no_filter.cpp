/*
 * @Description  : no filter model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:51:36
 */

#include "models/cloud_filter/no_filter.hpp"

#include <glog/logging.h>

namespace localization
{
NoFilter::NoFilter()
{
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr)
{
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
}  // namespace localization