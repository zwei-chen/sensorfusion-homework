/*
 * @Description  : 常用
 * @Author       : zhiwei chen
 * @Date         : 2022-07-17 13:55:25
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 17:57:24
 */
#pragma once
// 多线程相关
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <thread>

// 文件流相关
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

// 数据结构相关
#include <memory>
#include <vector>

// 姿态变量相关
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/src/Geometry/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
// 日志
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

// ros相关
#include <ros/callback_queue.h>
#include <ros/ros.h>

// ros data
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Geo
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

// 本地数据
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/pose_data.hpp"

// tool
#include "tools/tic_toc/tic_toc.hpp"

#define G_ 9.79125

// Filter Special Function
enum FILTER_FUNCTION
{
    SETSIZE,
    SETORIGIN,
    GETEDGE
};

// Registration type choose
enum REG_TYPE
{
    NDT,
    HNU
};

inline double rad2deg(const double& radians)
{
    return radians * 180.0 / M_PI;
}

inline double deg2rad(const double& degrees)
{
    return degrees * M_PI / 180.0;
}

inline double RightHand2LeftHand(const double& right_hand_angle)
{
    return std::fmod((450.0 - right_hand_angle), 360.0);
}

inline double RightFrontUp2FrontLeftUp(const double& angle)
{
    return fmod((angle + 270.0), 360.0);
}

inline tf::Quaternion tfRPY2Quaterniond(const double& roll, const double& pitch, const double& yaw)
{
    tf::Quaternion output_quaterniond;
    output_quaterniond.setRPY(roll, pitch, yaw);
    return output_quaterniond;
}

inline Eigen::Matrix3d eigenRPY2Matrix3d(const double& roll_deg, const double& pitch_deg, const double& yaw_deg)
{
    Eigen::Vector3d eulerAngle(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
    return static_cast<Eigen::Matrix3d>(yawAngle * pitchAngle * rollAngle);
}

inline Eigen::Quaterniond eigenRPY2Quat(const double& roll_deg, const double& pitch_deg, const double& yaw_deg)
{
    Eigen::Vector3d eulerAngle(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
    return static_cast<Eigen::Quaterniond>(yawAngle * pitchAngle * rollAngle);
}

/**
 * @brief  保存位姿
 * @param  ofs ofstream&
 * @param  pose Matrix4d&
 * @return
 */
inline bool SavePose(const Eigen::Matrix4d& pose, std::ofstream& ofs)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ofs << pose(i, j);

            if (i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }

    return true;
}

inline bool SavePose(const Eigen::Matrix4d& pose, const double times, std::ofstream& ofs)
{
    for (int i = 0; i < 3; ++i)
    {
        ofs << times << " ";
        for (int j = 0; j < 4; ++j)
        {
            ofs << pose(i, j);

            if (i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }

    return true;
}

inline bool SavePose(const double& x, const double& y, const double& z, std::ofstream& ofs)
{
    ofs << std::fixed << std::setprecision(15) << 1 << " " << 0 << " " << 0 << " " << x << " "
        << 0 << " " << 1 << " " << 0 << " " << y << " "
        << 0 << " " << 0 << " " << 1 << " " << z << std::endl;

    return true;
}
