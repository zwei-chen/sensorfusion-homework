<!--
 * @Description  : 
 * @Author       : zhiwei chen
 * @Date         : 2022-09-12 11:38:12
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2022-09-12 22:59:43
-->
# 多传感器融合第二期大作业
## 1. 标定
标定测试使用的数据集是 **Campus dataset (large)**
运行提示，没法进行体素滤波
```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset.
```
将体素滤波器的参数变成0.5，可以正常进行体素滤波
结果为
```
result euler angle(RPY) : -0.078183 0.0473181 0.0525925
result extrinsic rotation matrix : 
    0.9975 -0.0560969   0.042985
 0.0525095   0.995373  0.0804743
-0.0473005 -0.0780159   0.995829
```
与文件的内容接近，说明标定结果正确
```
  extrinsicRPY: [1, 0, 0,
                  0, 1, 0,
                  0, 0, 1]
```
## 2. 使用建立的地图进行定位
由于时间和精力有限，这里使用NDT作为当前帧与地图匹配的方式
整体思路如下
1. 载入地图后，根据初始姿态划分局部地图，针对每一帧的点云数据与局部地图匹配获得匹配的姿态
2. 根据当前姿态维护一个局部地图，每当当前位置快接近局部地图时，以此时的位置区更新局部地图
3. 初始姿态需要额外提供（实际应用可以使用GNSS，ScanContext等方式恢复，这里给的是直接写在程序里面）

由于使用lio-sam建立的地图运行基于NDT的定位未能有一个很好效果,个人分析原因如下，但是没太多时间优化，这里使用自己的数据完成本功能
1. 使用kitti数据集，初始的速度变化大，且采用匀速运动模型，对初始状态没有一个很好的估计，导致NDT跟丢。猜想加入IMU信息能改善这一点。

运行
```
roslaunch localization localization.launch
```

具体的数据集，所使用的地图，及运行效果见网盘