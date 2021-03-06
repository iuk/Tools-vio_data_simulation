//
// Created by hyj on 18-1-19.
//

#include "imu.h"

#include <random>

#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles) {
  double roll  = eulerAngles(0);
  double pitch = eulerAngles(1);
  double yaw   = eulerAngles(2);

  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  Eigen::Matrix3d RIb;
  RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
      sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
      -sp, cp * sr, cp * cr;
  return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles) {
  double roll  = eulerAngles(0);
  double pitch = eulerAngles(1);

  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);

  Eigen::Matrix3d R;
  R << 1, 0, -sp,
      0, cr, sr * cp,
      0, -sr, cr * cp;

  return R;
}

IMU::IMU(Param p) : param_(p) {
  gyro_bias_ = Eigen::Vector3d::Zero();
  acc_bias_  = Eigen::Vector3d::Zero();
}

// 生成噪声
void IMU::addIMUnoise(MotionData& data) {
  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0.0, 1.0);

  // N(0,1) 随机变量
  Eigen::Vector3d noise_gyro(noise(generator_), noise(generator_), noise(generator_));

  // double gyro_noise_sigma = 0.015;  //连续时间噪声密度的标准差，单位 (rad/s) * (1/√hz)
  // 标准差 3x3 对角矩阵
  Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();

  // 向理想陀螺仪数据中加入噪声和偏置
  data.imu_gyro                          // 加入噪声和偏置之后的IMU 测量值
      = data.imu_gyro                    // 理想IMU测量值
        + (gyro_sqrt_cov                 // 连续时间-IMU 高斯白噪声-标准差 3x3 对角矩阵
           / sqrt(param_.imu_timestep))  // √采样时间 Δt=5ms
              * noise_gyro               // N(0,1) 随机变量
        + gyro_bias_;                    // 偏置值

  Eigen::Vector3d noise_acc(noise(generator_), noise(generator_), noise(generator_));
  Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();

  // 生成带噪声的 IMU 测量数据
  data.imu_acc = data.imu_acc                                              //
                 + (acc_sqrt_cov / sqrt(param_.imu_timestep)) * noise_acc  //
                 + acc_bias_;

  // gyro_bias update
  Eigen::Vector3d noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
  gyro_bias_                               // 本时刻 bias
      = gyro_bias_                         // 上一时刻 bias
        + param_.gyro_bias_sigma           // 连续时间-偏置变化率-标准差
              * sqrt(param_.imu_timestep)  // √采样时间
              * noise_gyro_bias;           // N(0.1) 随机变量
  data.imu_gyro_bias = gyro_bias_;

  // acc_bias update
  Eigen::Vector3d noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));
  acc_bias_ = acc_bias_ +
              param_.acc_bias_sigma * sqrt(param_.imu_timestep) * noise_acc_bias;
  data.imu_acc_bias = acc_bias_;
}

// 从运动模型中获取 某时刻的 运动数据(MotionData)
MotionData IMU::MotionModel(double t) {
  MotionData data;

  {
    // 静止模型
    data.imu_gyro     = Eigen::Vector3d(0,0,0);
    data.imu_acc      = Eigen::Vector3d(0,0,9.81);
    data.Rwb          = Eigen::MatrixXd::Identity(3,3);
    data.twb          = Eigen::Vector3d(0,0,0);
    data.imu_velocity = Eigen::Vector3d(0,0,0);
    data.timestamp    = t;
    return data;
  }

  // param
  float ellipse_x = 15;
  float ellipse_y = 20;
  float z         = 1;          // z轴做sin运动
  float K1        = 10;         // z轴的正弦频率是x，y的k1倍
  float K         = M_PI / 10;  // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

  // === 位置变化模型 ===
  // translation
  // twb:  body frame in world frame
  // 世界系下 机体系 位置坐标
  Eigen::Vector3d position(ellipse_x * cos(K * t) + 5,
                           ellipse_y * sin(K * t) + 5,
                           z * sin(K1 * K * t) + 5);

  // position导数　in world frame 
  // 速度
  Eigen::Vector3d dp(-K * ellipse_x * sin(K * t),
                     K * ellipse_y * cos(K * t),
                     z * K1 * K * cos(K1 * K * t));
  
  // position二阶导数
  // 加速度
  double K2 = K * K;
  Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t),
                      -K2 * ellipse_y * sin(K * t),
                      -z * K1 * K1 * K2 * sin(K1 * K * t));
  
  // === 姿态变化模型 === rpy 角与时间的关系
  // Rotation
  double k_roll  = 0.1;
  double k_pitch = 0.2;
  // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
  Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);    
  // euler angles 的导数
  // rpy 角的时间导数
  Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), K);  

  //    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
  //    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

  // body frame to world frame
  Eigen::Matrix3d Rwb      = euler2Rotation(eulerAngles);
  //  euler rates trans to body gyro
  Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;  

  //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
  Eigen::Vector3d gn(0, 0, -9.81);                         
  //  Rbw * Rwn * gn = gs
  Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn);  

  data.imu_gyro     = imu_gyro;
  data.imu_acc      = imu_acc;
  data.Rwb          = Rwb;
  data.twb          = position;
  data.imu_velocity = dp;
  data.timestamp    = t;
  return data;
}

//读取生成的imu数据并用imu运动学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist) {
  // 读入 IMU 测量数据
  std::vector<MotionData> imudata;
  LoadPose(src, imudata);

  std::ofstream save_points;
  save_points.open(dist);

  double dt           = param_.imu_timestep;
  Eigen::Vector3d Pwb = init_twb_;      // position :    from  imu measurements
  Eigen::Quaterniond Qwb(init_Rwb_);    // quaterniond:  from imu measurements
  Eigen::Vector3d Vw = init_velocity_;  // velocity  :   from imu measurements
  Eigen::Vector3d gw(0, 0, -9.81);      // ENU frame
  Eigen::Vector3d temp_a;
  Eigen::Vector3d theta;

  // 遍历 IMU 测量数据
  for (int i = 1; i < imudata.size(); ++i) {
    MotionData imupose = imudata[i];

    {
      // === IMU 位姿积分，基于欧拉积分 ===
      // 1. 计算世界系加速度，依据世界系姿态
      Eigen::Vector3d gw(0, 0, -9.81);  // ENU frame
      Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;
      // 2. 更新世界系位置 P
      Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
      // 3. 更新世界系速度 V
      Vw = Vw + acc_w * dt;
      // 4. 计算四元数旋转小量，基于三轴角速度
      Eigen::Quaterniond dq;
      // 旋转小量 ω·Δt = Δθ ，旋转小量 Δq = [1,½Δθ]
      Eigen::Vector3d dtheta_half = imupose.imu_gyro * dt / 2.0;
      dq.w()                      = 1;
      dq.x()                      = dtheta_half.x();
      dq.y()                      = dtheta_half.y();
      dq.z()                      = dtheta_half.z();
      dq.normalize();
      // 5. 更新世界系姿态 Q
      Qwb = Qwb * dq;
    }

    // 中值积分

    //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
    save_points << imupose.timestamp << " "
                << Qwb.w() << " "
                << Qwb.x() << " "
                << Qwb.y() << " "
                << Qwb.z() << " "
                << Pwb(0) << " "
                << Pwb(1) << " "
                << Pwb(2) << " "
                << imupose.imu_gyro(0) << " "
                << imupose.imu_gyro(1) << " "
                << imupose.imu_gyro(2) << " "
                << imupose.imu_acc(0) << " "
                << imupose.imu_acc(1) << " "
                << imupose.imu_acc(2) << " "
                << std::endl;
  }

  std::cout << "test　end" << std::endl;
}
