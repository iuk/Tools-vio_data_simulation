//
// Created by hyj on 18-1-19.
//

#ifndef IMUSIMWITHPOINTLINE_IMU_H
#define IMUSIMWITHPOINTLINE_IMU_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "param.h"

struct MotionData {
  // eigen 对齐 new 方法宏定义
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  // 世界系下机体系的 R t
  Eigen::Matrix3d Rwb;
  Eigen::Vector3d twb;

  // 陀螺仪 和 加速度计 读数(带有偏置和白噪声)
  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_gyro;

  //陀螺仪 和 加速度计 偏置(带有误差)
  Eigen::Vector3d imu_gyro_bias;
  Eigen::Vector3d imu_acc_bias;

  // 世界系下机体系的速度
  Eigen::Vector3d imu_velocity;
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);

class IMU {
 public:
  IMU(Param p);
  Param param_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d acc_bias_;

  Eigen::Vector3d init_velocity_;
  Eigen::Vector3d init_twb_;
  Eigen::Matrix3d init_Rwb_;

  MotionData MotionModel(double t);

  void addIMUnoise(MotionData& data);
  void testImu(std::string src, std::string dist);  // imu数据进行积分，用来看imu轨迹
};

#endif  //IMUSIMWITHPOINTLINE_IMU_H
