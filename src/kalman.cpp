/*
 * @Description: 卡尔曼预测
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-10-30 00:10:23
 * @LastEditors: Barimu
 * @LastEditTime: 2023-01-15 16:06:46
 */
#ifndef RM2022_KALMAN_CPP
#define RM2022_KALMAN_CPP

#include "../include/kalman.h"

using namespace std;
using namespace cv;

/**
 * @brief: 卡尔曼相关数据初始化
 * @param {KalmanFilter} m_KF
 */
kalman ::kalman()
{
    // 定义KalmanFilter类并初始化
    KalmanFilter KK(m_stateNum, m_measureNum, 0);

    m_KF = KK; // 自身卡尔曼滤波器对象赋值

    // 定义测量值   图像矩阵
    m_measurement = Mat::zeros(m_measureNum,
                               1,
                               CV_32F);

    // 转移矩阵 A
    m_KF.transitionMatrix = (Mat_<float>(m_stateNum,
                                         m_stateNum)
                                 << 1,
                             0, 1, 0,
                             0, 1, 0, 1,
                             0, 0, 1, 0,
                             0, 0, 0, 1);

    // 将下面几个矩阵设置为对角阵
    setIdentity(m_KF.measurementMatrix);                      // 测量矩阵 H
    setIdentity(m_KF.processNoiseCov, Scalar::all(1e-3));     // 过程噪声 Q
    setIdentity(m_KF.measurementNoiseCov, Scalar::all(1e-2)); // 测量噪声 R
    setIdentity(m_KF.errorCovPost, Scalar::all(1));           // 最小均方误差 Pt
    // randn( m_KF.statePost, Scalar::all(0), Scalar::all(0.1) );
    randn(m_KF.statePost, Scalar::all(0), Scalar::all(0.1)); // x(0)初始化
}

// 重新初始化滤波器状态
void kalman ::reInit(KalmanFilter &KF)
{
    // 将下面几个矩阵设置为对角阵
    setIdentity(m_KF.measurementMatrix);                      // 测量矩阵 H
    setIdentity(m_KF.processNoiseCov, Scalar::all(1e-3));     // 过程噪声 Q
    setIdentity(m_KF.measurementNoiseCov, Scalar::all(1e-2)); // 测量噪声 R
    setIdentity(m_KF.errorCovPost, Scalar::all(1));           // 最小均方误差 Pt
    // randn( m_KF.statePost, Scalar::all(0), Scalar::all(0.1) );
    randn(m_KF.statePost, Scalar::all(0), Scalar::all(0.1)); // x(0)初始化
}

/**
 * @description: 卡尔曼预测装甲板中心点
 * @param {float} x
 * @param {float} y
 * @return {*}
 * @author: Barimu
 */
cv::Point2f kalman::kal(float x, float y)
{
    cv::Point2f center;
    m_prediction = m_KF.predict();  // 进行一次预测 调用了卡尔曼滤波器的预测函数
    m_measurement.at<float>(0) = x; // 写入真实值
    m_measurement.at<float>(1) = y;
    for (int i = 1; i <= 5; i++) // 迭代5次
    {
        m_KF.correct(m_measurement);   // 更新 用测量值矩阵m_measurement对当前状态进行更新correct()
        m_prediction = m_KF.predict(); // 再次进行预测predict(),得到新预测值
        m_measurement.at<float>(0) = m_prediction.at<float>(0);
        m_measurement.at<float>(1) = m_prediction.at<float>(1); // 将预测结果写入测量矩阵m_measurement,进行下一次迭代
    }
    // 循环迭代5次后,从预测结果矩阵m_prediction中取出滤波后的值
    center.x = m_prediction.at<float>(0);
    center.y = m_prediction.at<float>(1);
    // 返回滤波预测的中心坐标点
    return center;
}

/**
 * @brief 调用卡尔曼预测
 * @param {armors} &finalarmor
 * @param {Mat} originFrame
 * @param {kalman} &m_k
 */
void kalman::predict(vgd_stl::armors &finalarmor, Mat originFrame)
{
    cv::Point2f centers = finalarmor.center;                              // 从检测到的装甲板finalarmor中获取中心坐标centers
    cv::Point predict_pt = kal(centers.x, centers.y);                     // 将centers作为测量输入,调用kal()函数进行卡尔曼滤波预测,得到预测坐标predict_pt
    cv::circle(originFrame, predict_pt, 3, cv::Scalar(34, 255, 255), -1); // 在原始图像originFrame上画出预测坐标predict_pt,绘制为一个小圆点
    // 对每一帧的装甲板目标均进行该预测过程
}
#endif