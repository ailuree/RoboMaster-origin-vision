// #include "./hikvision/include/camera.hpp"
#include "./hikvision/include/HaiKangCamera.h"
#include "./include/inference.h"
#include "./include/kalman.h"
#include "./include/kalmanfilter.h"
#include "./include/predict.h"
#include "./include/SolvePnP.h"
#include "./include/record.h"

#include <vector>
#include <mutex>
#include <thread>
#include <cmath>
#include "./include/serial.h"
#define NX
#define red // 宏定义了要识别的装甲板颜色
// #define blue                                //detect enemy color
// #define camera_record                       //相机记录

std::vector<std::string> g_colorName = {
    "BLUE", "RED", "NONE"}; // 定义一个颜色名称容器数组

std::vector<std::string> g_armorNumber = {
    "SENTRY", "NO1", "NO2", "NO3", "NO4", "NO5", "OUTPOST", "BASE"}; // 装甲板编号

// 在图像上绘制装甲板
void draw(cv::Mat &input, std::vector<vgd_detect::ArmorObject> Objects)
{
    // 遍历所有检测到的装甲板
    for (auto obj : Objects)
    {
        // 绘制装甲板的4条边
        cv::line(input, obj.pts[0], obj.pts[2],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        cv::line(input, obj.pts[1], obj.pts[3],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        cv::line(input, obj.pts[0], obj.pts[1],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        cv::line(input, obj.pts[2], obj.pts[3],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        cv::line(input, obj.pts[0], obj.pts[3],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        cv::line(input, obj.pts[1], obj.pts[2],
                 cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                            255,
                            obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                 2);
        // 将装甲板颜色转化为数组索引
        int tmp1 = -1;
        if (obj.color == vgd_stl::ArmorColor::RED)
            tmp1 = 1;
        else if (obj.color == vgd_stl::ArmorColor::BLUE)
            tmp1 = 0;
        else
            tmp1 = 2;
        // 将装甲板编号转化为数组索引
        int tmp2 = -1;
        if (obj.number == vgd_stl::ArmorNumber::SENTRY)
            tmp2 = 0;
        else if (obj.number == vgd_stl::ArmorNumber::NO1)
            tmp2 = 1;
        else if (obj.number == vgd_stl::ArmorNumber::NO2)
            tmp2 = 2;
        else if (obj.number == vgd_stl::ArmorNumber::NO3)
            tmp2 = 3;
        else if (obj.number == vgd_stl::ArmorNumber::NO4)
            tmp2 = 4;
        else if (obj.number == vgd_stl::ArmorNumber::NO5)
            tmp2 = 5;
        else if (obj.number == vgd_stl::ArmorNumber::OUTPOST)
            tmp2 = 6;
        else if (obj.number == vgd_stl::ArmorNumber::BASE)
            tmp2 = 7;
        // 在装甲板上写颜色 编号 置信度
        cv::putText(input, g_colorName[tmp1] + " " + g_armorNumber[tmp2] + " " + std::to_string(obj.prob), obj.pts[0], 1, 1,

                    cv::Scalar(obj.color == vgd_stl::ArmorColor::BLUE ? 255 : 0,
                               255,
                               obj.color == vgd_stl::ArmorColor::RED ? 255 : 0),
                    2);
    }
}

int main()
{
#ifdef NX
    // 初始化串口对象 设置串口参数
    Serial uart(BR115200, WORDLENGTH_8B, STOPBITS_1, PARITY_NONE, HWCONTROL_NONE);
    // uart.sOpen("/dev/ttyUSB0");
    // 打开串口
    uart.sOpen("/dev/CH340_usb");
#endif

#ifdef camera_record
    // 初始化原始图像和处理过后的图像录像对象
    record original_record;
    record after_record;
    original_record.init_record(0);
    after_record.init_record(1);
#endif
start_get_img:
    // 获取开始时间戳
    std::unique_lock<std::mutex> lock(mtx); // 设置互斥锁
    std::chrono::_V2::steady_clock::time_point time_start;
    // 定义图像矩阵
    cv::Mat frame, output;
    // 初始化相机对象
    HaiKangCamera HaiKang;
    // 启动设备
    HaiKang.StartDevice(0);

    // set resolution ratio  设置分辨率
    HaiKang.SetResolution(1280, 1024);
    // update time_start  更新时间戳偏移量
    HaiKang.UpdateTimestampOffset(time_start);
    // start collect frame 开始采集图像
    HaiKang.SetStreamOn();
    // set exposure level  设置曝光时间
    HaiKang.SetExposureTime(3500);
    // option 1
    // HaiKang.SetGAIN(0, 16);
    // HaiKang.SetGAIN(1, 8);
    // HaiKang.SetGAIN(2, 8);
    HaiKang.SetGAIN(3, 16); // 设置图像增益
    // is auto white balance using
    // HaiKang.Set_Auto_BALANCE();  //这里是设置自动白平衡
    // manual白平衡 BGR->012  设置白平衡 手动
    HaiKang.Set_BALANCE(0, 1930);
    HaiKang.Set_BALANCE(1, 1024);
    HaiKang.Set_BALANCE(2, 1022);
    // 初始化openVINO模型检测器
    vgd_detect::OpenVINODetector detector(
        "/home/intelnuc/Documents/vgd_rm2023_vision_nn/model/yolox.onnx",
        "CPU");
    detector.init();

    // 初始化角度预测对象
    angle::predict predict;
    predict.initialize();
    predict.target.id = -1;

    while (true)
    {
        // 计算运行时间
        double t = (double)cv::getTickCount();
        // 获取图像
        auto HaiKang_stauts = HaiKang.GetMat(frame);
        if (!HaiKang_stauts)
        {
            goto start_get_img;
        }
        // cv::resize(frame, frame, cv::Size(0, 0), 0.5, 0.5);
        // 复制原始输入图像
        auto input = frame.clone();

        if (input.empty())
            continue;
        // cv::resize(input, input,cv::Size(),0.5, 0.5);

        // those code if for find Armor don't remove them.
        // 对输入图像进行目标检测
        auto result = detector.push_input(input, 0);
        result.get();
        auto Objects = detector.getObjects();
        // the end of find Armor code is here.
        // 绘制检测结果
        output = input.clone();
        draw(output, Objects);
        // 初始化最远目标和标志位
        int min = 0;
        int flag = -1;
        // cout<<Objects.size()<<endl;
        // 如果没有检测到目标，清空追踪目标
        if (Objects.size() == 0)
            predict.target.id = -1;
        // 统计检测到的红蓝目标数量
        int sum = 0;
        for (int k = 0; k < Objects.size(); k++)
        {
#ifdef red
            // 统计红色目标
            if (Objects[k].color == vgd_stl::ArmorColor::RED || Objects[k].color == vgd_stl::ArmorColor::NONE)
                sum++;
#endif
#ifdef blue
            // 统计蓝色目标
            if (Objects[k].color == vgd_stl::ArmorColor::BLUE || Objects[k].color == vgd_stl::ArmorColor::NONE)
                sum++;
#endif
        }
        // 如果未检测到红蓝目标 清空追踪目标
        if (sum == 0)
            predict.target.id = -1;

        std::vector<bool> same_id(Objects.size(), false);
        int hi = 0;
        int hii = -1;
        // 对每个目标进行处理
        for (int j = 0; j < Objects.size(); j++)
        {
            // 将目标编号转化为数组下标
            int tmp2 = -1;
            if (Objects[j].number == vgd_stl::ArmorNumber::SENTRY)
                tmp2 = 0;
            else if (Objects[j].number == vgd_stl::ArmorNumber::NO1)
                tmp2 = 1;
            else if (Objects[j].number == vgd_stl::ArmorNumber::NO2)
                tmp2 = 2;
            else if (Objects[j].number == vgd_stl::ArmorNumber::NO3)
                tmp2 = 3;
            else if (Objects[j].number == vgd_stl::ArmorNumber::NO4)
                tmp2 = 4;
            else if (Objects[j].number == vgd_stl::ArmorNumber::NO5)
                tmp2 = 5;
            if (tmp2 == -1)
                continue;
            Objects[j].num = tmp2;
#ifdef red
            // if(Objects[j].color!=vgd_stl::ArmorColor::RED)continue;
            // 只保留红色目标
            if (Objects[j].color == vgd_stl::ArmorColor::BLUE)
                continue;
#endif

#ifdef blue
            // 只保留蓝色目标
            if (Objects[j].color == vgd_stl::ArmorColor::RED)
                continue;
#endif
            //        if(Objects.size()!=0&&hi==0)
        }
        // 对保留下来的目标进行距离估计
        if (!Objects.empty())
        {
            for (int i = 0; i < Objects.size(); i++)
            {
                // cout<<"ok"<<endl;
                // cout<<predict.target.id<<endl;
#ifdef red
                // 只处理红色目标
                if (Objects[i].color != vgd_stl::ArmorColor::RED)
                    continue;
#endif

#ifdef blue
                // 只处理蓝色目标
                if (Objects[i].color != vgd_stl::ArmorColor::BLUE)
                    continue;
#endif

                hii++;
                if (Objects[i].num == -1)
                    continue;
                // 转换为目标结构体
                vgd_stl::ArmorObject nearCenterArmor = Objects[i];
                vgd_stl::armors Armor;
                // 填充目标角点
                Armor.corner[1] = nearCenterArmor.pts[0];
                Armor.corner[2] = nearCenterArmor.pts[1];
                Armor.corner[3] = nearCenterArmor.pts[2];
                Armor.corner[4] = nearCenterArmor.pts[3];
                // 填充目标中心点
                Armor.number = Objects[i].num;
                Armor.center.x = Objects[i].box.x + cvRound(Objects[i].box.width / 2.0);
                Armor.center.y = Objects[i].box.y + cvRound(Objects[i].box.height / 2.0);
                Armor.length = vgd_stl::getDistance(Objects[i].pts[0], Objects[i].pts[1]);
                Armor.boardw = vgd_stl::getDistance(Objects[i].pts[0], Objects[i].pts[1]);
                Armor.boardh = vgd_stl::getDistance(Objects[i].pts[0], Objects[i].pts[3]);
                // cv::circle(output, Point2i (frame.cols/2.0,frame.rows/2.0), 3, cv::Scalar(34, 255, 255), -1);
                // 绘制中心点
                cv::circle(output, Point2i(frame.cols / 2.0, frame.rows / 2.0), 15, cv::Scalar(0, 0, 255), 4);
                // 初始化解算PNP
                SOLVEPNP pnp;

                pnp.midx = frame.cols / 2.0;
                pnp.midy = frame.rows / 2.0;
                // 调用pnp算法计算坐标
                pnp.caculate(Armor);
                // 保存目标位姿
                Objects[i].position[0] = Armor.position[0];
                Objects[i].position[1] = Armor.position[1];
                Objects[i].position[2] = Armor.position[2];
                // 更新最近和最远目标
                if (hii == 0)
                {
                    min = Armor.dtm;
                    flag = i;
                }
                else
                {
                    if (min > Armor.dtm)
                    {
                        min = Armor.dtm;
                        flag = i;
                    }
                }
            }
            // 选择最近目标
            if (flag != -1 && Objects[flag].position[2] <= 1000)
            {
                vgd_stl::armors finalArmor;
                // 填充最近目标信息
                finalArmor.position[0] = Objects[flag].position[0];
                finalArmor.position[1] = Objects[flag].position[1];
                finalArmor.position[2] = Objects[flag].position[2];
                // 计算俯仰角和偏航角
                double pitch = atan(finalArmor.position[1] / finalArmor.position[2]) / CV_PI * 180;
                double yaw = atan(finalArmor.position[0] / finalArmor.position[2]) / CV_PI * 180;
                //                cout << "pitch=" << pitch << endl;
                //                cout << "yaw=" << yaw << endl << endl;
                //                cout << finalArmor.position[0] << endl << finalArmor.position[1] << endl << finalArmor.position[2]
                //                     << endl << endl;
                // 填充目标编号
                finalArmor.number = Objects[flag].num;
                // 追踪最近目标
                predict.armortrack(finalArmor);

                // #ifdef NX
                //            //if(tmp > 10)
                //             uart.sSendData(yaw, pitch,finalArmor.position[2],1);
                // #endif
            }
            // #ifdef NX
            // if(tmp > 10)
            //             uart.sSendData(yaw, pitch,finalArmor.position[2],1);//发送串口数据                                                                                                                                                                                                                                                                                                                                                   
            // #endif
        }
        // 计算处理时间
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // 计算并打印fps
        int fps = int(1.0 / t);
        std::cout << "FPS: " << fps << std::endl;
#ifdef camera_record
        // 录制原始和初始的视频
        original_record.start_record(input);
        after_record.start_record(output);
#endif
        cv::imshow("output", output); // 显示处理后的图像
        if (cv::waitKey(1) == 27)
        {
            return 0;
        } // 检测退出键推出
    }

    return 0;
}
