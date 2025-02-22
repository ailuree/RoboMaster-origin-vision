/*
 * @Description:
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-10-30 00:10:23
 * @LastEditors: Barimu
 * @LastEditTime: 2022-12-05 11:55:11
 */

#ifndef RM2022_SOLVEPNP_H
#define RM2022_SOLVEPNP_H

#include "opencv2/opencv.hpp"
#include "vgd_standard_content.hpp"
#include <queue>
#include <vector>
#include <iostream>

#ifdef NX
#include "serial.h"
#endif

using namespace std;
using namespace cv;

/**
 * @description: pnp类
 */
// 求解相机外参
class SOLVEPNP
{
public:
    vector<Point2d> picture_points; // 相机坐标下的四个点

    float distance;

    int midx;
    int midy;
    // 1. distance: 相机到目标的距离,会在计算过程中得到
    // 2. midx: 目标在图像坐标系中的x坐标
    // 3. midy: 目标在图像坐标系中的y坐标

    // 通过给 点对 求取旋转平移矩阵的函数
    float PNP(vgd_stl::armors &finalarmor, int flag);
    // 计算装甲板姿态的函数
    void caculate(vgd_stl::armors &finalarmor);
    // 通过旋转矩阵计算相对角度的函数
    void calAngle(Mat cam, Mat dis, int x, int y);

    float xishu;

    double yaw;
    double pitch;
    double rxNew;
    double ryNew;
    // 1. xishu: 用于存储一个缩放系数,后续计算中会用到
    // 2. yaw: 计算得到的目标相对于相机的偏航角,绕Z轴旋转角度
    // 3. pitch: 计算得到的目标相对于相机的俯仰角,绕X轴旋转角度
    // 4. rxNew: 目标在相机坐标系下的X轴坐标
    // 5. ryNew: 目标在相机坐标系下的Y轴坐标

private:
    vector<Point3d> model_points; // 存储实际空间下的坐标点

    // 相机内参矩阵
    Mat camera_matrix = (Mat_<double>(3, 3) << 1.201371857055914e+03, 0, 7.494419594994199e+02,
                         0, 1.201435954410725e+03, 5.508546827593877e+02,
                         0, 0, 1);

    // 相机畸变系数
    Mat dist_coeffs = (Mat_<double>(5, 1) << -0.098380553375716, 0.006115203108383,
                       -4.766609631726518e-04, -0.001862163979558, 0);

    // 旋转向量
    Mat rotation_vector;

    // 平移向量
    Mat translation_vector;

    const double P = 3.1415926;
};
#endif