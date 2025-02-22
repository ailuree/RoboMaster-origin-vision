/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-12-05 20:45:49
 * @LastEditors: Barimu
 * @LastEditTime: 2023-01-15 12:15:31
 */
#include"../include/predict.h"
#include"../include/serial.h"
#include "../include/armor_tracker.h"
#include "../include/SolvePnP.h"
#define NX
namespace angle
{
    //初始化跟踪预测模块中的卡尔曼滤波器的参数矩阵
    void predict::initialize()
    {	
        // Kalman Filter initial matrix
        // A - state transition matrix
        // clang-format off
        // 定义状态转移矩阵A
        Eigen::Matrix<double, 6, 6> f;
        // 构建A矩阵
        // 矩阵第一行表示位置的自回归
        // 矩阵第四行表示速度的自回归
        f <<  1,  0,  0, dt_, 0,  0,
                0,  1,  0,  0, dt_, 0,
                0,  0,  1,  0,  0, dt_,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;
        // clang-format on

        // H - measurement matrix
        //// 定义测量矩阵H
        Eigen::Matrix<double, 3, 6> h;
        // 构建H矩阵,单位矩阵 设成单位阵
        h.setIdentity();

        // Q - process noise covariance matrix
        // 定义过程噪声方差矩阵Q
        Eigen::DiagonalMatrix<double, 6> q;
        //// 在位置分量设置较小噪声,速度分量设置较大噪声
        q.diagonal() << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1;

        // R - measurement noise covariance matrix
        // 定义观测噪声方差矩阵R
        Eigen::DiagonalMatrix<double, 3> r;
        // 设置较小的观测噪声
        r.diagonal() << 0.05, 0.05, 0.05;

        // P - error estimate covariance matrix
        // 定义误差方差矩阵P
        Eigen::DiagonalMatrix<double, 6> p;
        // 初始化为单位阵
        p.setIdentity();

        // 构建参数结构体
        kf_matrices_ = KalmanFilterMatrices{f, h, q, r, p};

        // Tracker
        // 初始化跟踪器参数
        double max_match_distance =  0.2;
        int tracking_threshold =  5;
        int lost_threshold =  5;
        last_time=0.0;
        dt_=0.0;
        //下面是tracker对象
        tracker_ =  std::unique_ptr<ArmorTracker>(new ArmorTracker(kf_matrices_, max_match_distance,
                     tracking_threshold, lost_threshold));
    }

    //跟踪装甲板
    void predict::armortrack(vgd_stl::armors armor)
    {	
#ifdef NX
        //初始化串口通信
    Serial uart(BR115200, WORDLENGTH_8B, STOPBITS_1, PARITY_NONE, HWCONTROL_NONE);
    //uart.sOpen("/dev/ttyUSB0");
    uart.sOpen("/dev/CH340_usb");//打开的端口名
#endif

        // 如果跟踪状态为丢失
        if (tracker_->tracker_state == ArmorTracker::LOST) 
        {
            // 重新初始化跟踪
            tracker_->init(armor);
            // 记录当前时间
            last_time=clock();
            // 目标跟踪状态设置为false
            target.tracking = false;
        } 
        else 
        {   
            // Set dt
            // 计算两帧间的时间间隔
            dt_ = (clock() - last_time)/CLOCKS_PER_SEC;
            // 记录当前时间
            last_time=clock();
            // Update state
            //卡尔曼预测更新
            tracker_->update(armor, dt_);
        }
        // 如果检测到目标
        if (tracker_->tracker_state == ArmorTracker::DETECTING)
        {
            // 目标跟踪状态设置为false
            target.tracking = false;
        }
            // 如果正在跟踪或暂时丢失
        else if (
            tracker_->tracker_state == ArmorTracker::TRACKING ||
            tracker_->tracker_state == ArmorTracker::TEMP_LOST) 
        {
            // 设置正在跟踪
            target.tracking = true;
            // 获取目标ID
            target.id = tracker_->tracking_id;
        }
        // 如果正在跟踪
        if (target.tracking)
        {
            // 更新目标状态
            target.position[0] = tracker_->target_state(0);
            target.position[1] = tracker_->target_state(1);
            target.position[2] = tracker_->target_state(2);
            target.velocity[0] = tracker_->target_state(3);
            target.velocity[1] = tracker_->target_state(4);
            target.velocity[2] = tracker_->target_state(5);
            target.id = tracker_->target_state(6);
            //double pitchpredict=atan(target.position[1]/target.position[2])/CV_PI*180;
            //double yawpredict=atan(target.position[0]/target.position[2])/CV_PI*180;
            // 计算欧拉角
            double pitchreal=atan(armor.position[1]/armor.position[2])/CV_PI*180;
            double yawreal=atan(armor.position[0]/armor.position[2])/CV_PI*180;                                         
            //cout<<"predictpitch="<<pitchpredict<<endl;
            //cout<<"predictyaw="<<yawpredict<<endl<<endl;
            // 显示输出
             cout<<"realpitch="<<pitchreal<<endl;
             cout<<"realyaw="<<yawreal<<endl<<endl;
#ifdef NX
            //if(tmp > 10)
            // 发送串口数据
            uart.sSendData(yawreal, pitchreal,target.position[2],1);
            //cout<<"111"<<endl;
#endif
        }
    }
}
