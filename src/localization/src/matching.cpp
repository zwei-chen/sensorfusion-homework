/*
 * @Description  : Matching Localization
 * @Author       : zhiwei chen
 * @Date         : 2022-07-16 22:07:24
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 22:53:43
 */
#define PCL_NO_PRECOMPILE
#include "matching.hpp"

#include "models/cloud_filter/box_filter.hpp"
#include "models/cloud_filter/voxel_filter.hpp"
#include "models/registration/ndt_registration.hpp"

namespace localization
{
Matching::Matching(const YAML::Node& config_node)
    : global_map_ptr_(new CloudData::CLOUD()),
      local_map_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD())
{
    InitWithConfig(config_node);

    InitGlobalMap(config_node);

    // Init new thread
    // localmap_thread_prt_ = std::make_shared<std::thread>(&Matching::UpdateLocalMap, this);
}

bool Matching::InitWithConfig(const YAML::Node& config_node)
{
    registration_ptr_ = std::make_shared<NDTRegistration>(config_node["NDT"]);
    // a. global map filter -- downsample point cloud map for visualization:
    global_map_filter_ptr_ = std::make_shared<VoxelFilter>(config_node["FILTER_VOXEL"]["global_map"]);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    local_map_filter_ptr_ = std::make_shared<VoxelFilter>(config_node["FILTER_VOXEL"]["local_map"]);
    // c. scan filter --
    frame_filter_ptr_ = std::make_shared<VoxelFilter>(config_node["FILTER_VOXEL"]["frame"]);

    return true;
}

bool Matching::InitGlobalMap(const YAML::Node& config_node)
{
    std::string map_path_ = config_node["map_path"].as<std::string>();

    // load map:
    LOG(INFO) << "Load global map begin";
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

    // since scan-map matching is used, here apply the same filter to local map & scan:
    // because local map have some downsampled like map build,now no useful
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z)
{
    // base on orientation to change point
    std::vector<float> origin = {x, y, z};

    // use ROI filtering for local map segmentation:
    box_filter_ptr_->FilterSet(origin, SETORIGIN);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    // set local map is matching cloud point
    registration_ptr_->SetInputTarget(local_map_ptr_);

    // show box filter edge
    std::vector<float> edge;
    box_filter_ptr_->FilterSet(edge, GETEDGE);
    LOG(INFO) << "New local map:" << edge.at(0) << ","
              << edge.at(1) << ","
              << edge.at(2) << ","
              << edge.at(3) << ","
              << edge.at(4) << ","
              << edge.at(5) << std::endl
              << std::endl;

    return true;
}

bool Matching::JudgeLocalMap(const Eigen::Matrix4d& laser_pose)
{
    static int reset_map_count = 0;

    //! judge local map whether update
    std::vector<float> edge;
    box_filter_ptr_->FilterSet(edge, GETEDGE);
    ++reset_map_count;

    // use distance update
    for (int i = 0; i < 3; i++)
    {
        if (fabs(laser_pose(i, 3) - edge.at(2 * i)) > 150.0 && fabs(laser_pose(i, 3) - edge.at(2 * i + 1)) > 150.0)
        {
            continue;
        }

        current_localmap_pose_ = laser_pose;
        ResetLocalMap(current_localmap_pose_(0, 3), current_localmap_pose_(1, 3), current_localmap_pose_(2, 3));
        break;
    }

    // use fitness score update
    if (registration_ptr_->GetFitnessScore() > 0.5 && reset_map_count > 10)
    {
        LOG(INFO) << "fitness score large, reset local map";
        current_localmap_pose_ = laser_pose;
        ResetLocalMap(current_localmap_pose_(0, 3), current_localmap_pose_(1, 3), current_localmap_pose_(2, 3));
        reset_map_count = 0;
    }
    return true;
}

// void Matching::UpdateLocalMap()
// {
//     while (ros::ok())
//     {
//         std::unique_lock<std::mutex> lock(localmap_mtx_);
//         LOG(INFO) << "Update LocalMap";

//         localmap_cv_.wait(lock);
//         lock.unlock();
//         ResetLocalMap(current_localmap_pose_(0, 3), current_localmap_pose_(1, 3), current_localmap_pose_(2, 3));
//     }
// }

bool Matching::Update(const CloudData& cloud_data,
                      Eigen::Matrix4d& laser_pose)
{
    TicToc tictoc;
    static Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
    static Eigen::Matrix4d last_pose = init_pose_;
    static Eigen::Matrix4d predict_pose = init_pose_;

    //! a. remove invalid measurements:
    std::vector<int> indices;
    cloud_data.cloud_ptr->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    //! b. downsample current scan:
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

    //! c. matching:
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, laser_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, laser_pose);

    //! d. update predicted pose and local map
    step_pose = last_pose.inverse().matrix() * laser_pose;
    predict_pose = laser_pose * step_pose;
    last_pose = laser_pose;

    JudgeLocalMap(laser_pose);

    // prompt:
    LOG(INFO) << "------ Finish Matching -------" << std::endl
              << "Time Used: " << tictoc.toc() << " seconds." << std::endl
              << "Cost Reduced: " << registration_ptr_->GetFitnessScore() << std::endl
              << std::endl;

    return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4d& init_pose)
{
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

    return true;
}

CloudData::CLOUD_PTR& Matching::GetGlobalMap()
{
    return global_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan()
{
    return current_scan_ptr_;
}

}  // namespace localization