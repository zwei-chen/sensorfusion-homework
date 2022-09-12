/*
 * @Description  : NDT registration model
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 18:04:33
 */

#define PCL_NO_PRECOMPILE
#include "models/registration/ndt_registration.hpp"

namespace localization
{
NDTRegistration::NDTRegistration(const YAML::Node& node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
{
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
{
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
{
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "NDT params:";
    LOG(INFO) << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target)
{
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                Eigen::Matrix4d& predict_pose,
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4d& result_pose)
{
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose.cast<float>());
    result_pose = (ndt_ptr_->getFinalTransformation()).cast<double>();

    return true;
}

float NDTRegistration::GetFitnessScore()
{
    return ndt_ptr_->getFitnessScore();
}

}  // namespace localization