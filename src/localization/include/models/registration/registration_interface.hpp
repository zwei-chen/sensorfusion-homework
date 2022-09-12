/*
 * @Description  : Cloud points interface
 * @Author       : zhiwei chen
 * @Date         : 2022-06-14 23:24:16
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:54:41
 */
#pragma once

#include "common/common.h"

namespace localization
{
class RegistrationInterface
{
  public:
    RegistrationInterface() = default;
    virtual ~RegistrationInterface() = default;

    /**
     * @brief scan-to-map matching 设置需要匹配到的点云，一般为局部地图
     * @param input_target input map
     * @return true
     */
    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;

    /**
     * @brief 计算点云信息之间的位姿变换
     * @param input_source 需要匹配的点云，一般为当前扫描
     * @param predict_pose 先验位姿
     * @param result_cloud_ptr 需要匹配的点云进行位姿变换后的坐标
     * @param result_pose 两个点云之间的位姿变换
     * @return true
     */
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                           Eigen::Matrix4d& predict_pose,
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4d& result_pose) = 0;

    /**
     * @brief 获得点云信息之间的误差
     * @return 点云信息之间的误差
     */
    virtual float GetFitnessScore() = 0;
};
}  // namespace localization
