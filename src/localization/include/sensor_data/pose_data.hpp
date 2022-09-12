/*
 * @Description  : Processed Pose
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:04:17
 */

#pragma once

#include "common/common.h"

namespace localization
{
class VelocityData;
class PoseData
{
  public:
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    double time = 0.0;

    struct
    {
        Eigen::Vector3f v = Eigen::Vector3f::Zero();
        Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;

  public:
    Eigen::Quaterniond GetQuaternion();
};
}  // namespace localization
