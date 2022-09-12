/*
 * @Description  : NDT registration model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 18:02:51
 */
#pragma once
#include <pcl/registration/ndt.h>

#include "models/registration/registration_interface.hpp"

namespace localization
{
class NDTRegistration : public RegistrationInterface
{
  public:
    NDTRegistration(const YAML::Node& node);
    /**
     * @brief  构造该类需要的参数
     * @param res 网络分辨率
     * @param step_size 步长
     * @param trans_eps 终止条件设置的最小转换差异
     * @param max_iter 最大迭代次数
     */
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                   Eigen::Matrix4d& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4d& result_pose) override;
    float GetFitnessScore() override;

  private:
    /**
     * @brief  设置NDT匹配的参数
     * @param res 网络分辨率
     * @param step_size 步长
     * @param trans_eps 终止条件设置的最小转换差异
     * @param max_iter 最大迭代次数
     * @return  true
     */
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};
}  // namespace fuse_localization
