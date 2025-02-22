/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-12-05 10:59:25
 * @LastEditors: Barimu
 * @LastEditTime: 2022-12-05 11:55:08
 */
/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-11-25 10:56:16
 * @LastEditors: Barimu
 * @LastEditTime: 2022-11-25 11:57:05
 */
#ifndef ARMOR_PROCESSOR__KALMANFILTER_H
#define ARMOR_PROCESSOR__KALMANFILTER_H

#include"../include/Eigen/Dense"

namespace angle
{
    //KalmanFilterMatrices结构体:存储滤波器所需的矩阵F,H,Q,R,P
struct KalmanFilterMatrices
{
    Eigen::MatrixXd F;  // state transition matrix 状态转移矩阵
    Eigen::MatrixXd H;  // measurement matrix 测量矩阵
    Eigen::MatrixXd Q;  // process noise covariance matrix 过程噪声协方差
    Eigen::MatrixXd R;  // measurement noise covariance matrix 测量噪声协方差
    Eigen::MatrixXd P;  // error estimate covariance matrix 先验/后验误差协方差矩阵
};

class KalmanFilter
{
public:
    explicit KalmanFilter(const KalmanFilterMatrices & matrices);

    // Initialize the filter with a guess for initial states.
    //初始化滤波器状态
    void init(const Eigen::VectorXd & x0);

    // Computes a predicted state
    //进行预测
    Eigen::MatrixXd predict(const Eigen::MatrixXd & F);

    // Update the estimated state based on measurement
    //利用新测量进行更新
    Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
    // Invariant matrices
    Eigen::MatrixXd F, H, Q, R;

    // Priori error estimate covariance matrix
    Eigen::MatrixXd P_pre;//先验估计误差协方差矩阵,预测步骤中计算
    // Posteriori error estimate covariance matrix
    Eigen::MatrixXd P_post;//后验估计误差协方差矩阵,更新步骤中计算

    // Kalman gain 卡尔曼增益 反映新测量对状态修正的权重
    Eigen::MatrixXd K;

    // System dimensions
    int n;//系统状态维度

    // N-size identity
     Eigen::MatrixXd I; //n阶单位矩阵

    // Predicted state
    Eigen::VectorXd x_pre; //先验状态向量
    // Updated state
    Eigen::VectorXd x_post;//后验状态向量
};

}
#endif
