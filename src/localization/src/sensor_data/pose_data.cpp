/*
 * @Description  : Processed Pose
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:04:05
 */

#include "sensor_data/pose_data.hpp"

namespace localization
{
Eigen::Quaterniond PoseData::GetQuaternion()
{
    Eigen::Quaterniond q;
    q = pose.block<3, 3>(0, 0);

    return q;
}

}  // namespace localization