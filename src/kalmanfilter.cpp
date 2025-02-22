/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-11-25 10:56:25
 * @LastEditors: Barimu
 * @LastEditTime: 2022-12-05 12:29:46
 */
#include"../include/kalmanfilter.h"

namespace angle
{

KalmanFilter::KalmanFilter(const KalmanFilterMatrices & matrices):
  F(matrices.F),
  H(matrices.H),
  Q(matrices.Q),
  R(matrices.R),
  P_post(matrices.P),
  n(matrices.F.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pre(n),
  x_post(n)
{
}
//用给定的初始状态向量x0初始化状态向量
void KalmanFilter::init(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd KalmanFilter::predict(const Eigen::MatrixXd & F)
{
  this->F = F;

  //- 计算先验状态向量x_pre
  x_pre = F * x_post;
  //- 计算先验误差协方差矩阵P_pre
  P_pre = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pre;
  P_post = P_pre;
//- 如果没有新的测量,先验结果作为后验结果输出
  return x_pre;
}

Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & z)
{
    //- 计算卡尔曼增益K
  K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
  //- 计算后验状态向量x_post
  x_post = x_pre + K * (z - H * x_pre);
  //- 计算后验误差协方差矩阵P_post
  P_post = (I - K * H) * P_pre;

  return x_post;
}

}