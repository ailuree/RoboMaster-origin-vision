/*
 * @Description: 卡尔曼预测
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-10-30 00:10:23
 * @LastEditors: ZeraTul ZeraTttul@gmail.com
 * @LastEditTime: 2022-11-02 00:43:34
 */

// todo: 改构造函数
#ifndef RM2022_KALMAN_H
#define RM2022_KALMAN_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "../include/SolvePnP.h"
#include "vgd_standard_content.hpp"
#include <queue>
#include <vector>
#include <iostream>


/**
 * @brief: 卡尔曼类
 */
class kalman
{
private:
    const int m_stateNum = 4;                          //状态值4×1向量(x,y,△x,△y)
    const int m_measureNum = 2;                        //测量值2×1向量(x,y)
    //   - m_stateNum状态量为4维,包括x,y,x的变化量,y的变化量
    //    - m_measureNum测量量为2维,即测量得到的x,y
    cv::Mat m_measurement;                                 //测量值
    cv::Mat m_prediction;                                  //预测值
public:
    cv::KalmanFilter m_KF;//卡尔曼滤波器对象
    kalman();
    void reInit(KalmanFilter &KF);//重新初始化滤波器状态
    void predict(vgd_stl::armors &finalarmor, cv::Mat originframe);//进行卡尔曼预测
    cv::Point2f kal(float x, float y);//对单个测量进行滤波
};
#endif