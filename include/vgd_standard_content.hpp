//
// Created by intelnuc on 23-4-4.
//

#ifndef VGD_RM2023_VISION_NN_VGD_STANDARD_CONTENT_HPP
#define VGD_RM2023_VISION_NN_VGD_STANDARD_CONTENT_HPP

#include <opencv2/opencv.hpp>
#include <cmath>

//这里的命名空间是 vgd_stl
namespace vgd_stl {
    // primitive armor
    //原始图像处理时的装甲板信息
    struct armors {
        float length;                    //装甲板长
        float boardw;                   //宽度
        float boardh;                   //高度
        cv::Point2f corner[5];               //装甲板四个角点   [1][3]为对角
        cv::Point2f center;                  //装甲板中心点
        float position[3];               //现实坐标x,y,z
        int number;                  //装甲板上数字


        double dtm;     //？？？

    };

    enum class ArmorColor {
        RED = 0,
        BLUE,
        NONE,
        PURPLE//紫色？
    };
    enum class ArmorNumber {
        SENTRY = 0,//哨兵
        NO1,
        NO2,
        NO3,
        NO4,
        NO5,
        OUTPOST,//前哨战
        BASE//基地
    };
    // neural network armor
    //即通过神经网络识别到的装甲板对象
    struct ArmorObject{
        ArmorColor color;
        ArmorNumber number;//装甲板显示数字
        float prob;//识别后的置信度分数
        std::vector<cv::Point2f> pts;//装甲板的轮廓点坐标集合
        cv::Rect box;//装甲板的外接矩形框
        float position[3]; //装甲板的三维坐标
        int num;//识别到的装甲板编号
    };

    static double getDistance(cv::Point2f pts1, cv::Point2f pts2) {
        return std::sqrt((pts1.x - pts2.x)*(pts1.x - pts2.x) + (pts1.y - pts2.y) * (pts1.y - pts2.y));
    }
}

#endif //VGD_RM2023_VISION_NN_VGD_STANDARD_CONTENT_HPP
