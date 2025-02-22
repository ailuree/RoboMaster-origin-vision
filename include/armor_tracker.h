#ifndef _ARMOR_TRACKER_H_
#define _ARMOR_TRACKER_H_

#include "vgd_standard_content.hpp"
#include "kalman.h"
#include "../include/kalmanfilter.h"
#include <queue>


//这里的命名空间是angle
namespace angle
{
    
class ArmorTracker
{
    private:
        angle::KalmanFilterMatrices kf_matrices_;  //卡尔曼滤波的参数矩阵
        std::unique_ptr<angle::KalmanFilter> kf_;  //卡尔曼滤波器指针
        Eigen::Vector3d tracking_velocity_;             //跟踪的装甲板速度
        double max_match_distance_;                     //匹配的最大距离
    public:
        ArmorTracker(const angle::KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
  int lost_threshold);    
        void init(const vgd_stl::armors &armor);//初始化装甲板目标
        void update(const vgd_stl::armors &armor, const double & dt);//更新每一帧的跟踪

        Eigen::VectorXd target_state;//目标的状态向量

        //  - 对每一帧,检测到装甲板后,更新到跟踪器
        //  - 跟踪器利用卡尔曼滤波进行预测和更新
        //  - 根据阈值判断目标跟踪状态
        //  - 输出跟踪的目标状态
        //实现了一个基于卡尔曼滤波的闭环装甲板跟踪器。

        int tracking_id;//跟踪id
        enum State 
        {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        } tracker_state;//跟踪器状态
        int tracking_threshold_;//跟踪阈值
        int lost_threshold_;//丢失阈值

        int detect_count_;//检测计数
        int lost_count_;//丢失计数
};
}

#endif
