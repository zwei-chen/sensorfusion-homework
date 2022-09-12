/*
 * @Description  : Cloud point data
 * @Author       : zhiwei chen
 * @Date         : 2022-06-14 23:24:16
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 18:25:24
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/common.h"
#include "sensor_data/pcl_point_types/point_types.h"

namespace localization
{
class CloudData
{
  public:
    // Include intensity,ring and time
    // using POINT = PointXYZIRT;
    using POINT = pcl::PointXYZ;

    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
        : cloud_ptr(new CLOUD())
    {
    }

  public:
    // 时间戳
    double time = 0.0;

    // 点云数据
    CLOUD_PTR cloud_ptr;
};
}  // namespace localization