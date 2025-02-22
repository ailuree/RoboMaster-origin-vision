#ifndef INFERENCE
#define INFERENCE
//推理解算
#include "../include/Eigen/Dense"

#include "vgd_standard_content.hpp"

#include <string>
#include <vector>
#include <future>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <opencv2/core/mat.hpp>

namespace vgd_detect {
    using ArmorObject = vgd_stl::ArmorObject;
//结构体存储特征图的网格大小和特征层步长
    struct GridAndStride {
        int grid0;
        int grid1;
        int stride;
    };
//grid0 和 grid1 表示特征图的网格大小,一般是模型输出特征图的大小。
//stride 表示这个特征层的步长。
//在目标检测模型中,一般会有多个不同步长的特征层,用于检测不同大小的对象。每个特征层都可以用一个 GridAndStride 来表示。
//在后处理中,需要根据 GridAndStride 中的信息,将模型输出的预测框还原到原始图像坐标系下。
//具体来说,如果模型预测一个框的位置为 (grid_x, grid_y), 那么这个框在原始图像中的坐标为:
//bbox_x = grid_x * stride  利用步长得到原始坐标
//bbox_y = grid_y * stride

//openvino检测器
    class OpenVINODetector {
    public:
        /**
         * @brief Construct a new OpenVINO Detector object
         *
         * @param model_path     IR/ONNX file path  检测模型的文件 路径
         * @param device_name   Target device (CPU, GPU, AUTO) 运行的目标设备
         * @param conf_threshold  Confidence threshold for output filtering 置信度阈值 用来过滤低置信度的检测框
         * @param top_k  Topk parameter //topK参数 NMS过程中保留topk个框
         * @param nms_threshold  NMS threshold      NMS的阈值
         * @param auto_init  If initializing detector inplace 是否在构造时直接初始化检测器
         *
         * 可以看出,构造函数允许用户设置关键的模型、设备和后处理的参数。
            conf_threshold、top_k和nms_threshold组合用于过滤和排序预测框,去除重叠和置信度较低的框。
            auto_init参数则控制是否在构造时就加载模型并初始化,如果设置为false,需要用户手动调用init()初始化。

            NMS
            NMS(Non-Maximum Suppression) 非极大值抑制,是目标检测算法中的一个重要的后处理步骤。
            在目标检测模型的预测结果中,同一个目标可能会被预测出多个矩形框,这些框重叠度很高。NMS的目的是合并这些框,留下对同一个目标最佳的一个框。
            NMS的基本过程是:
            1. 将预测框按置信度从高到低排序。
            2. 从中选取置信度最高的框A,输出为最终检测结果。
            3. 计算其余框与A的IoU(交并比),去除IoU大于阈值的框。
            4. 重复步骤2、3,直到Boxes为空。
            通过迭代的移除重叠框操作,NMS可以有效地保留对每个目标最佳的单个框,移除冗余的重复框。
            NMS过程中两个重要的参数是置信度阈值和IoU阈值,一般需要经过调整取得最佳效果。
            NMS是目标检测的标准流程,YOLO、Faster R-CNN等算法都采用不同形式的NMS后处理。它可以明显提升检测精度。
         */
         //explicit关键字作用
         //指定构造函数或转换函数 (C++11起)为显式, 即它不能用于隐式转换和复制初始化.
         //explicit 指定符可以与常量表达式一同使用. 函数若且唯若该常量表达式求值为 true 才为显式. (C++20起)
        explicit OpenVINODetector(//构造
                const std::string model_path,
                const std::string & device_name,
                float conf_threshold = 0.25, int top_k = 128, float nms_threshold = 0.3,
                bool auto_init = false);//这里的构造是 带缺省值的
        /**
         * @brief Initialize detector
         */
        void init();

        /**
         * @brief Push a single image to inference 获取一张图像来做推理
         *
         * @param rgb_img  //输入的RGB图像,使用OpenCV的Mat类型。
         * @param timestamp_nanosec //该帧图像的时间戳,纳秒级
         * @return std::future<bool>  If callback finished return ture. 进程执行完毕之后返回1
         */
        std::future<bool> push_input(const cv::Mat & rgb_img, int64_t timestamp_nanosec);
        //- 返回一个std::future对象,用于异步获取推理结果。future对象封装了一个异步任务的状态,可以查询任务是否完成或获取结果。
        //方法使用future对象实现了异步推理:
        //1. 将图像输入到推理任务中。
        //2. 返回一个future对象。
        //3. 实际的推理计算在后台线程进行。
        //4. 调用方可以根据future判断任务完成与否,或者在完成后获取结果。
        //5. 不需要等待推理完成就可以进行后续处理。
        //这样可以 Pipeline 推理与后处理,充分利用CPU/GPU并行计算能力,提升总体吞吐量

    private:
        bool process_callback(
                const cv::Mat resized_img,
                Eigen::Matrix3f transform_matrix,
                int64_t timestamp_nanosec,
                const cv::Mat & src_img);

    private:
        std::string model_path_;
        std::string device_name_;
        float conf_threshold_;
        int top_k_;
        float nms_threshold_;
        std::vector<int> strides_;//模型中各个特征层的步长
        std::vector<GridAndStride> grid_strides_;//存储各个特征层的网格大小和步长。


        std::unique_ptr<ov::Core> ov_core_;//OpenVINO核心类指针
        std::unique_ptr<ov::CompiledModel> compiled_model_;//OpenVINO编译模型指针
        std::vector<ArmorObject> m_objs;//存储检测结果的向量


    public:
        std::vector<ArmorObject> getObjects(); //返回检测结果m_objs
    };
}  // namespace vgd_detect

#endif  // INFERENCE
