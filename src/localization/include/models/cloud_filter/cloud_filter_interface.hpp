/*
 * @Description  : cloud filter interface
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:50:30
 */
#pragma once

#include <glog/logging.h>
#include <pcl/filters/filter.h>
#include <yaml-cpp/yaml.h>

#include "sensor_data/cloud_data.hpp"

namespace localization
{
class CloudFilterInterface
{
  public:
    virtual ~CloudFilterInterface() = default;

    /**
     * @brief  对点云执行滤波操作
     * @param input_cloud_ptr 输入点云
     * @param filtered_cloud_ptr  输出点云
     * @return true
     */
    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                        CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;

    /**
     * @brief  子类滤波器特殊函数
     * @param  input 输入数据
     * @param  function_type 执行功能
     * @return void
     */
    virtual void FilterSet(std::vector<float>& input, int function_type) { LOG(ERROR) << "the class no use this function"; }
};
}  // namespace localization
