/*
 * @Description  : Matching Localization
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 18:13:05
 */

#pragma once
#define PCL_NO_PRECOMPILE

#include "common/common.h"
#include "models/cloud_filter/cloud_filter_interface.hpp"
#include "models/registration/registration_interface.hpp"

namespace localization
{
class Matching
{
  public:
    Matching(const YAML::Node& config_node);

    /**
     * @brief  进行点云匹配并输出位姿
     * @param  cloud_data 当前点云，用于输入进行匹配
     * @param  laser_pose 进行匹配后的位姿
     * @return true
     */
    bool Update(const CloudData& cloud_data,
                Eigen::Matrix4d& laser_pose);

    /**
     * @brief  设置初始化位姿并重设局部地图
     * @param  init_pose 初始化位姿
     * @return true
     */
    bool SetInitPose(const Eigen::Matrix4d& init_pose);

    /**
     * @brief  获取全局地图点云
     * @param  void
     * @return 全局地图
     */
    CloudData::CLOUD_PTR& GetGlobalMap();

        /**
     * @brief  获取当前帧点云
     * @param  void
     * @return 当前帧
     */
    CloudData::CLOUD_PTR& GetCurrentScan();

  private:
    /**
     * @brief  模块初始化相关
     *         依次执行
     *         InitScanContextManager: 初始化回环检测的数据和方法
     *         InitGlobalMap: 初始化全局地图，从文件载入之前生成的地图
     * @param config_node 配置文件
     * @param filter_user 滤波器接口
     * @return 正常为true，否则为false
     */
    bool InitWithConfig(const YAML::Node& config_node);
    bool InitGlobalMap(const YAML::Node& config_node);

    /**
     * @brief  根据ROI区域和相应坐标，重设局部地图
     * @param  x 坐标x
     * @param  y 坐标y
     * @param  z 坐标z
     * @return true
     */
    bool JudgeLocalMap(const Eigen::Matrix4d& laser_pose);
    bool ResetLocalMap(float x, float y, float z);

    void UpdateLocalMap();
    

  private:
    // Registration
    std::shared_ptr<RegistrationInterface> registration_ptr_;

    // Filter
    // Map filter
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
    // local filter
    std::shared_ptr<CloudFilterInterface> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    // scan filter
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    // Thread
    std::shared_ptr<std::thread> localmap_thread_prt_;

    // Lock and Condition Variable
    std::condition_variable localmap_cv_;
    std::mutex localmap_mtx_;

    // Cloud points
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    // Pose
    Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d current_gnss_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d current_localmap_pose_ = Eigen::Matrix4d::Identity();
};
}  // namespace localization
