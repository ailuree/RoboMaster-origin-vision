/*
 * @Description: 
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-12-05 12:34:59
 * @LastEditors: Barimu
 * @LastEditTime: 2022-12-05 22:19:44
 */
#ifndef PREDICT_H
#define PREDICT_H

#include<string>
#include<vector>
#include "vgd_standard_content.hpp"
#include"../include/kalmanfilter.h"
#include"../include/armor_tracker.h"

namespace angle {
    class targets {
        public:
            bool tracking; //目标的跟踪状态
            double position[3];//位置
            double velocity[3];//速度
            int id;
    };

    class predict {
        public:
            void initialize();//initialize(): 初始化
            void armortrack(vgd_stl::armors armor);//对检测到的装甲板进行跟踪预测
            targets target;//存储跟踪的目标状态
        private:

             // Initial KF matrices
            KalmanFilterMatrices kf_matrices_;//- kf_matrices_: 卡尔曼滤波的参数矩阵

            // Armor tracker
            std::unique_ptr<ArmorTracker> tracker_;//ArmorTracker跟踪器对象指针

            //TODO:add angle
            double last_time; //上一帧时间
            double dt_;  //两帧间时间间隔
    };
}
#endif