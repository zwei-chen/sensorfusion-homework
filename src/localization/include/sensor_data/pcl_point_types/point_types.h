/*
 * @Description  : point cloud type
 * @Author       : zhiwei chen
 * @Date         : 2022-06-14 23:24:16
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-07-14 19:52:08
 */
#ifndef LOM_POINT_TYPES_H_
#define LOM_POINT_TYPES_H_

#include <pcl/point_types.h>

/** Euclidean coordinate, including intensity and ring number. */
struct PointXYZIR
{
    PCL_ADD_POINT4D;  // quad-word XYZ
    PCL_ADD_INTENSITY;
    std::uint16_t ring;              /// laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZIRT
{
    PCL_ADD_POINT4D;  // quad-word XYZ
    std::uint8_t intensity;
    std::uint16_t ring;  /// laser ring number
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZIT
{
    PCL_ADD_POINT4D;  // quad-word XYZ
    double timestamp;
    uint8_t intensity;
    uint8_t flags;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(std::uint8_t, intensity, intensity)(std::uint8_t, flags, flags))
#endif  // LOM_POINT_TYPES_H_
