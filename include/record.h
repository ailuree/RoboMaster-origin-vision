//
// Created by intelnuc on 23-6-6.
//

#ifndef VGD_RM2023_VISION_NN_RECORD_H
#define VGD_RM2023_VISION_NN_RECORD_H
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<iostream>

using namespace std;
using namespace cv;

class record
{
public:
    void init_record(int model);
    void start_record(Mat frame);
private:
    VideoWriter outputVideo;
    string path;
    int fps =60;
    Size S = Size(1280,1024);
};
#endif //VGD_RM2023_VISION_NN_RECORD_H
