/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-12-05 10:58:35
 * @LastEditors: Barimu
 * @LastEditTime: 2023-03-06 21:37:47
 */
#include"../include/predict.h"
#include<cfloat>
#include<iostream>

namespace angle
{

ArmorTracker::ArmorTracker(const KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
  int lost_threshold)
: tracker_state(LOST),
  tracking_id(0),
  kf_matrices_(kf_matrices),
  tracking_velocity_(Eigen::Vector3d::Zero()),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold),
  detect_count_(0),
  lost_count_(0)
{
}

void ArmorTracker::init(const vgd_stl::armors &armor)
{
    cout<<"1"<<endl;
  // KF init
    // 创建卡尔曼滤波器对象,初始化滤波器矩阵
  kf_ = std::unique_ptr<KalmanFilter>(new KalmanFilter(kf_matrices_));
    // 定义初始状态变量,6维向量
  Eigen::VectorXd init_state(6);
    // 获取检测到的装甲目标位置
  const auto position = armor.position;
    // 输出装甲目标位置
    cout<<position[0]<<" "<<position[1]<<" "<<position[2]<<endl;
    // 将位置坐标赋值给状态变量前3维
  init_state << position[0], position[1], position[2], 0, 0, 0;
    // 将状态变量传递给卡尔曼滤波器对象,完成初始化
  kf_->init(init_state);
    // 获取装甲目标编号,赋值给跟踪id
  tracking_id = armor.number;
    // 将跟踪状态设置为检测状态
  tracker_state = DETECTING;
}

void ArmorTracker::update(const vgd_stl::armors &armor, const double & dt)
{
  // KF predict
    // 用时间间隔dt更新滤波器矩阵
  kf_matrices_.F(0, 3) = kf_matrices_.F(1, 4) = kf_matrices_.F(2, 5) = dt;
    // 预测装甲目标状态
  Eigen::VectorXd kf_prediction = kf_->predict(kf_matrices_.F);

  bool matched = false;        //实际值与预测值是否匹配
  // Use KF prediction as default target state if no matched armor is found
    // 默认目标状态为预测状态
  target_state = kf_prediction;

    // 存储匹配的装甲目标
    vgd_stl::armors matched_armor;
    // 最小位置差,初始化为最大值
    double min_position_diff = DBL_MAX;
    // 当前装甲目标位置
    Eigen::Vector3d position_vec(armor.position[0], armor.position[1], armor.position[2]);
    // 预测的装甲目标位置
    Eigen::Vector3d predicted_position = kf_prediction.head(3);
    // Difference of the current armor position and tracked armor's predicted position
    // 当前和预测位置差异
    double position_diff = (predicted_position - position_vec).norm();
//    if (position_diff < min_position_diff)
//    {
        //min_position_diff = position_diff;
    // 存储匹配的装甲信息
        matched_armor = armor;
	//cout<<"matched"<<endl;
   // }
    // 如果检测到的目标与预测的目标匹配,更新滤波器
    if (true)
    {
      // Matching armor found
      matched = true;
      Eigen::Vector3d position_vec(
      matched_armor.position[0], matched_armor.position[1], matched_armor.position[2]);
      //cout<<matched_armor.position<<endl;
      //cout<<position_vec<<endl;

      target_state = kf_->update(position_vec);
      target_state(6)=armor.number;
    } else 
    {
        // 如果未匹配到,如果编号相同则重置滤波器
      if(tracking_id==armor.number)
      {
          matched = true;
          // Reset KF
          // 重置滤波器
          kf_ = std::unique_ptr<KalmanFilter>(new KalmanFilter(kf_matrices_));
          // 用当前装甲信息初始化
          Eigen::VectorXd init_state(6);
          // Set init state with current armor position and tracking velocity before
          init_state << armor.position[0], armor.position[1], armor.position[2], tracking_velocity_;
          kf_->init(init_state);
          // 更新目标状态
          target_state = init_state;
      }
    }
    //cout<<matched<<endl;
  // Save tracking target velocity
    // 存储目标速度
  tracking_velocity_ = target_state.tail(3);
  // Tracking state machine
    // 跟踪状态机
  if (tracker_state == DETECTING) {
    // DETECTING
    //cout<<detect_count_<<endl;
    if (matched) {
        // 递增检测计数
      detect_count_++;
        // 检测次数达阈值则转为跟踪状态
      if (detect_count_ > tracking_threshold_) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
        // 未匹配则为丢失状态
      detect_count_ = 0;
      tracker_state = LOST;
    }

      // 跟踪阶段
  } else if (tracker_state == TRACKING) {
    // TRACKING
    if (!matched) {
        // 未匹配则转为暂时丢失
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
    // 暂时丢失阶段
  } else if (tracker_state == TEMP_LOST) {
    // LOST
    if (!matched) {
        // 丢失次数递增
      lost_count_++;
        // 达阈值则转为完全丢失
      if (lost_count_ > lost_threshold_) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    }// 匹配成功则转为跟踪状态
    else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}
}
