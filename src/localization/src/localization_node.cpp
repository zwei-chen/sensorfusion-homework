/*
 * @Description  :
 * @Author       : zhiwei chen
 * @Date         : 2022-09-12 16:43:52
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 22:47:59
 */
#include <glog/logging.h>
#include <ros/ros.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "common/common.h"
#include "matching.hpp"

using namespace localization;

ros::Publisher odom_pub;
ros::Publisher cloud_pub;
ros::Publisher scan_pub;
ros::Subscriber cloud_sub;
tf::TransformBroadcaster *broadcaster;
std::shared_ptr<Matching> matching_ptr;

void call_back(const sensor_msgs::PointCloud2::ConstPtr &input_msg_ptr)
{
    //! 0. transform cloud points
    CloudData cloud_data;
    cloud_data.time = input_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*input_msg_ptr, *(cloud_data.cloud_ptr));

    //! 1. matching
    Eigen::Matrix4d laser_odometry_ = Eigen::Matrix4d::Identity();
    matching_ptr->Update(cloud_data, laser_odometry_);

    //! 2. publish
    nav_msgs::Odometry odometry_;
    odometry_.header.stamp = ros::Time::now();
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "/lidar";
    // set the position
    odometry_.pose.pose.position.x = laser_odometry_(0, 3);
    odometry_.pose.pose.position.y = laser_odometry_(1, 3);
    odometry_.pose.pose.position.z = laser_odometry_(2, 3);
    Eigen::Quaterniond q;
    q = laser_odometry_.block<3, 3>(0, 0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();
    odom_pub.publish(odometry_);

    // publish global map for show
    CloudData::CLOUD_PTR scan_ptr(new CloudData::CLOUD());
    scan_ptr = matching_ptr->GetCurrentScan();
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*scan_ptr, *cloud_ptr_output);
    ros::Time ros_time(ros::Time::now());
    cloud_ptr_output->header.stamp = ros_time;
    cloud_ptr_output->header.frame_id = "map";
    scan_pub.publish(*cloud_ptr_output);

    // publish tf
    tf::StampedTransform transform;
    transform.frame_id_ = "map";
    transform.child_frame_id_ = "/lidar";
    Eigen::Quaterniond q_tf(laser_odometry_.block<3, 3>(0, 0));
    transform.stamp_ = ros::Time::now();
    transform.setRotation(tf::Quaternion(q_tf.x(), q_tf.y(), q_tf.z(), q_tf.w()));
    transform.setOrigin(tf::Vector3(laser_odometry_(0, 3), laser_odometry_(1, 3), laser_odometry_(2, 3)));
    broadcaster->sendTransform(transform);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 0;
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/laser_odom", 1000000);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1000000);
    scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1000000);
    // TAG lio-sam 数据集的点云消息
    // cloud_sub = nh.subscribe("/points_raw", 1000000, call_back);
    // TAG 自己数据集的点云消息
    cloud_sub = nh.subscribe("/rslidar_points", 1000000, call_back);
    broadcaster = new tf::TransformBroadcaster();

    // init matching
    std::string config_file_path = "/home/zwei/workspace_docker/fuse_localization/workspace/sensorfusion-homework/src/localization/config/Matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    matching_ptr = std::make_shared<Matching>(config_node["param"]);

    // TAG lio-sam 数据及的初始姿态
    // // init pose
    // Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
    // Eigen::Matrix3d extRPY = Eigen::Matrix3d::Identity();
    // extRPY(0, 0) = 9.999976e-01;
    // extRPY(0, 1) = 7.553071e-04;
    // extRPY(0, 2) = -2.035826e-03;
    // extRPY(1, 0) = -7.854027e-04;
    // extRPY(1, 1) = 9.998898e-01;
    // extRPY(1, 2) = -1.482298e-02;
    // extRPY(2, 0) = 2.024406e-03;
    // extRPY(2, 1) = 1.482454e-02;
    // extRPY(2, 2) = 9.998881e-01;
    // Eigen::Quaterniond extQRPY = Eigen::Quaterniond(extRPY);
    // Eigen::Quaterniond q;
    // q.x() = 0.003140;
    // q.y() = 0.002731;
    // q.z() = -0.223907;
    // q.w() = 0.974602;
    // q = q * extQRPY;
    // init_pose.block<3, 3>(0, 0) = q.matrix();
    // matching_ptr->SetInitPose(init_pose);

    // TAG 自己数据及的初始姿态
    // init pose
    Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
    init_pose(0, 0) = 000.885424;
    init_pose(0, 1) = 0 - 0.463932;
    init_pose(0, 2) = 00.0281321;
    init_pose(0, 3) = 00 - 8914.34;
    init_pose(1, 0) = 000.464656;
    init_pose(1, 1) = 000.884978;
    init_pose(1, 2) = 0 - 0.030155;
    init_pose(1, 3) = 00 - 3075.61;
    init_pose(2, 0) = -0.0109064;
    init_pose(2, 1) = 00.0397717;
    init_pose(2, 2) = 000.999149;
    init_pose(2, 3) = 00034.8174;
    init_pose(3, 0) = 0;
    init_pose(3, 1) = 0;
    init_pose(3, 2) = 0;
    init_pose(3, 3) = 1;
    matching_ptr->SetInitPose(init_pose);

    ros::Rate rate(100);
    while (ros::ok())
    {
        static bool init_map = false;
        if (!init_map)
        {
            sleep(3);
            // publish global map for show
            CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
            global_map_ptr = matching_ptr->GetGlobalMap();
            LOG(INFO) << "map size is" << global_map_ptr->size();
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*global_map_ptr, *cloud_ptr_output);
            ros::Time ros_time(ros::Time::now());
            cloud_ptr_output->header.stamp = ros_time;
            cloud_ptr_output->header.frame_id = "map";
            cloud_pub.publish(*cloud_ptr_output);
            init_map = true;
        }

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}