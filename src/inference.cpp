#include <algorithm>

#include "../include/inference.h"


namespace vgd_detect {
    static const int INPUT_W = 416;        // Width of input
    static const int INPUT_H = 416;        // Height of input
    static constexpr int NUM_CLASSES = 8;  // Number of classes
    static constexpr int NUM_COLORS = 4;   // Number of color
    static constexpr float MERGE_CONF_ERROR = 0.15;
    static constexpr float MERGE_MIN_IOU = 0.9;
    //- INPUT_W 和 INPUT_H: 定义了模型输入图像的宽度和高度,这里是416x416。
    //- NUM_CLASSES: 定义了模型输出的数字类别数,这里是8
    //- NUM_COLORS: 定义了模型输出的颜色类别数,这里是4
    //- MERGE_CONF_ERROR: 在后处理中合并重叠框时使用的置信度阈值,如果重叠框的置信度相差在这个范围内,则合并。
    //- MERGE_MIN_IOU: 合并重叠框时使用的IOU阈值,IOU大于这个值的框会被合并。
    //所以这些常量定义了这个目标检测模型的一些关键参数,比如输入大小、类别数、后处理中的超参数等,是模型inference过程需要的配置信息。将它们定义为常量可以全局共享这些参数设置

    //在这下面又写了一些函数  并不在openvenoDetector这个类里 但后面可能要用

    //对输入图像进行填充(padding)和缩放(resize),将其reshape为规定大小(如416x416),同时计算坐标变换矩阵。
    static cv::Mat letterbox(
            const cv::Mat & img,
            Eigen::Matrix3f & transform_matrix,
            std::vector<int> new_shape = {INPUT_W, INPUT_H}) {
        //传参
        //1. const cv::Mat &img: 输入图像,是一个const的cv::Mat类型引用,也就是原始输入图像。
        //2. Eigen::Matrix3f &transform_matrix:坐标变换矩阵,是一个Eigen库的3x3 float矩阵引用,letterbox会计算这个矩阵并写入,提供原图到缩放后图像的坐标转换。
        //3. std::vector<int> new_shape:缩放目标尺寸,默认是{416, 416},是一个宽高的vector。
        // Get current image shape [height, width]
        //// 获取输入图像尺寸,rows为高,cols为宽
        int img_h = img.rows;
        int img_w = img.cols;

        // Compute scale ratio(new / old) and target resized shape
        //// 计算缩放比例scale和目标缩放后尺寸,使缩放图像最大限度适应new_shape
        double scale = std::min(new_shape[1] * 1.0 / img_h, new_shape[0] * 1.0 / img_w);
        int resize_h = static_cast<int>(round(img_h * scale));  //round函数实现四舍五入
        int resize_w = static_cast<int>(round(img_w * scale));

        // Compute padding
        // 计算需要padding的高度和宽度
        int pad_h = new_shape[1] - resize_h;
        int pad_w = new_shape[0] - resize_w;

        // Resize and pad image while meeting stride-multiple constraints
        //按比例缩放原图到计算所得尺寸
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

        // divide padding into 2 sides
        // 将padding量等分到图像上下左右两侧
        double half_h = pad_h * 1.0 / 2;
        double half_w = pad_w * 1.0 / 2;

        // Compute padding boarder
        // 计算上下左右的padding大小
        int top = static_cast<int>(round(half_h - 0.1));
        int bottom = static_cast<int>(round(half_h + 0.1));
        int left = static_cast<int>(round(half_w - 0.1));
        int right = static_cast<int>(round(half_w + 0.1));


        /* clang-format off */
        /* *INDENT-OFF* */

        // Compute point transform_matrix
        // 计算原图到缩放后图像的坐标变换矩阵
        transform_matrix << 1. / scale, 0, -half_w / scale,
                0, 1. / scale, -half_h / scale,
                0, 0, 1;

        /* *INDENT-ON* */
        /* clang-format on */


        // Add border
        // 添加填充边框实现letterbox  opencv库函数
        cv::copyMakeBorder(
                resized_img, resized_img,
                top, bottom, left, right,
                cv::BORDER_CONSTANT,
                cv::Scalar(114, 114,114));
        return resized_img;
    }

/**
 * @brief Generate grids and stride.
 * @param target_w Width of input.
 * @param target_h Height of input.
 * @param strides A vector of stride.
 * @param grid_strides Grid stride generated in this function.
 */
 //1. 输入参数:
 //    - target_w: 输入图像宽度
 //    - target_h: 输入图像高度
 //    - strides: 一组不同的步长设置
 //2. 输出参数:
 //   - grid_strides: 网格和步长信息的向量
 //3. 对每个步长stride:
 //   - 计算网格的宽度数目: num_grid_w = target_w / stride
 //   - 计算网格的高度数目: num_grid_h = target_h / stride
 //4. 嵌套循环遍历所有网格:
 //   - 对于每一行g1 = 0 ~ num_grid_h-1:
 //     - 对于每一列g0 = 0 ~ num_grid_w-1:
 //       - 将当前网格(g0, g1)和当前步长stride写入grid_strides

 //5. 最终grid_strides包含了输入图像按照不同步长划分的所有网格信息。

 //生成给定输入图像大小和步长的网格(grid)信息
    static void generate_grids_and_stride(
            const int target_w, const int target_h,
            std::vector<int> & strides,
            std::vector<GridAndStride> & grid_strides) {

        for (auto stride : strides) {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;

            for (int g1 = 0; g1 < num_grid_h; g1++) {
                for (int g0 = 0; g0 < num_grid_w; g0++) {
                    grid_strides.emplace_back(GridAndStride{g0, g1, stride});
                }
            }
        }
    }


//    1. 输入参数:
//    - output_buffer: YOLO模型的输出,包含每个cell的预测信息
//    - transform_matrix: 将预测框坐标转换到原图像尺寸的变换矩阵
//    - conf_threshold: 置信度阈值,用于过滤低置信度的预测
//    - grid_strides: 包含每个预测层的网格信息
//    2. 循环每个anchor:
//    - 计算置信度confidence,如果小于阈值则跳过
//    - 获取当前anchor的网格坐标grid0,grid1和步长stride
//    - 从output_buffer提取颜色分类confidence,取出置信度最高的颜色色class_id
//    - 同理提取数字类别confidence,取出置信度最高的数字class_id
//    - 使用grid0, grid1, stride和预测框坐标,转换到原图尺寸
//    - 应用变换矩阵transform_matrix到转换后的坐标
//    - 将框坐标points、class_id等信息组装成一个检测目标proposal
//    - 添加到输出列表
//    3. 最终生成了每一个anchor预测的检测框信息,包括置信度、坐标、类别等。
//    这样就完成了针对每个预测anchor,提取其预测信息,生成检测目标候选框的过程。这是基于YOLO模型输出的后处理操作

//从YOLO模型的输出中生成检测目标的候选框proposals proposal也可理解为候选框
    static void generate_proposals(
            std::vector<ArmorObject> & output_objs,//检测目标proposal的输出向量
            std::vector<float> & scores,//proposal的置信度得分输出向量
            std::vector<cv::Rect> & rects,//proposal的框坐标矩形输出向量
            const cv::Mat & output_buffer,
            const Eigen::Matrix<float, 3, 3> & transform_matrix,
            float conf_threshold,
            std::vector<GridAndStride> grid_strides) {

        const int num_anchors = grid_strides.size();//根据步长设定我的anchor也就是候选框的数目

        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {//对每个拿到的anchor进行处理
            float confidence = output_buffer.at<float>(anchor_idx, 8);//获取当前anchor的置信度
            //过滤低于阈值的置信度
            if (confidence < conf_threshold) {
                continue;
            }
            // 获取当前anchor的网格坐标
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            // 获取当前anchor的步长
            const int stride = grid_strides[anchor_idx].stride;
            // 从output_buffer提取颜色和数字的置信度
            double color_score, num_score;
            cv::Point color_id, num_id;
            cv::Mat color_scores = output_buffer.row(anchor_idx).colRange(9, 9 + NUM_COLORS);
            cv::Mat num_scores = output_buffer.row(anchor_idx).colRange(9 + NUM_COLORS, 9 + NUM_COLORS + NUM_CLASSES);
            // Argmax
            // 找到颜色和数字的最大置信类别
            cv::minMaxLoc(color_scores, NULL, &color_score, NULL, &color_id);
            cv::minMaxLoc(num_scores, NULL, &num_score, NULL, &num_id);

            // 根据grid和stride转换框的坐标到原图尺寸
            float x_1 = (output_buffer.at<float>(anchor_idx, 0) + grid0) * stride;
            float y_1 = (output_buffer.at<float>(anchor_idx, 1) + grid1) * stride;
            float x_2 = (output_buffer.at<float>(anchor_idx, 2) + grid0) * stride;
            float y_2 = (output_buffer.at<float>(anchor_idx, 3) + grid1) * stride;
            float x_3 = (output_buffer.at<float>(anchor_idx, 4) + grid0) * stride;
            float y_3 = (output_buffer.at<float>(anchor_idx, 5) + grid1) * stride;
            float x_4 = (output_buffer.at<float>(anchor_idx, 6) + grid0) * stride;
            float y_4 = (output_buffer.at<float>(anchor_idx, 7) + grid1) * stride;

            // 将坐标应用变换矩阵到原图
            Eigen::Matrix<float, 3, 4> apex_norm;
            Eigen::Matrix<float, 3, 4> apex_dst;


            /* clang-format off */
            /* *INDENT-OFF* */
            apex_norm << x_1, x_2, x_3, x_4,
                    y_1, y_2, y_3, y_4,
                    1,   1,   1,   1;
            /* *INDENT-ON* */
            /* clang-format on */


            apex_dst = transform_matrix * apex_norm;
            // 构建检测目标proposal
            ArmorObject obj;
            obj.pts.resize(4);
            // 填充坐标
            obj.pts[0] = cv::Point2f(apex_dst(0, 0), apex_dst(1, 0));
            obj.pts[1] = cv::Point2f(apex_dst(0, 1), apex_dst(1, 1));
            obj.pts[2] = cv::Point2f(apex_dst(0, 2), apex_dst(1, 2));
            obj.pts[3] = cv::Point2f(apex_dst(0, 3), apex_dst(1, 3));

            // 填充类别
            auto rect = cv::boundingRect(obj.pts);
            obj.box = rect;
            obj.color = static_cast<vgd_stl::ArmorColor>(color_id.x);
            obj.number = static_cast<vgd_stl::ArmorNumber>(num_id.x);
            //// 填充置信度
            obj.prob = confidence;
            // 添加到输出
            rects.push_back(rect);
            scores.push_back(confidence);
            output_objs.push_back(std::move(obj));
        }
    }


/**
 * @brief Calculate intersection area between two objects.
 * @param a Object a.
 * @param b Object b.
 * @return Area of intersection.
 */
    static inline float intersection_area(const ArmorObject & a, const ArmorObject & b) {
        cv::Rect_<float> inter = a.box & b.box;
        return inter.area();
    }
    //这个静态内联函数intersection_area用来计算两个检测框的交集面积。
    //实现步骤:
    //1. 使用cv::Rect的位与操作&计算两个框的交集矩形inter
    //2. 计算交集矩形inter的面积并返回
    //其中传参a和b为两个ArmorObject对象,包含了检测框信息。
    //利用OpenCV的Rect类可以非常方便地计算矩形框的交集,从而实现两个框的交集面积计算。
    //返回值float类型表示两个矩形框的交集面积大小。
    //这可以用于后续计算两个框的交并比IoU来判断框是否重合。这个函数作为一个基础工具函数被后面的nms函数调用

    //非极大值抑制(NMS)和合并检测框的功能:
    static void nms_merge_sorted_bboxes(
            std::vector<ArmorObject> & faceobjects,
            std::vector<int> & indices,
            float nms_threshold) {
        //传参 1. faceobjects:存储着所有检测到的ArmorObject对象,即检测框信息
        //2. indices: 用来存储保留下来的检测框索引的向量
        //3. nms_threshold:非极大抑制的阈值,用于判断检测框是否具有重复性
        indices.clear();

        const int n = faceobjects.size();

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++) {
            areas[i] = faceobjects[i].box.area();
        }

        for (int i = 0; i < n; i++) {
            ArmorObject & a = faceobjects[i];

            int keep = 1;
            for (size_t j = 0; j < indices.size(); j++) {
                ArmorObject & b = faceobjects[indices[j]];

                // intersection over union
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[indices[j]] - inter_area;
                float iou = inter_area / union_area;
                if (iou > nms_threshold || isnan(iou)) {
                    keep = 0;
                    // Stored for Merge
                    if (a.number == b.number && a.color == b.color &&
                        iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR)
                    {
                        for (int i = 0; i < 4; i++) {
                            b.pts.push_back(a.pts[i]);
                        }
                    }
                }
            }

            if (keep) {
                indices.push_back(i);
            }
        }
    }
//    1. 计算每个检测框的面积areas,存储在向量中
//    2. 遍历每个检测框a
//    3. 对于每个检测框a,与已经保留下来的检测框b进行比较
//    - 计算iou交并比
//    - 如果iou大于阈值nms_threshold,则跳过该检测框a
//    - 否则,保留该检测框a
//    4. 如果两个框满足:
//    - 类别相同
//    - 置信度相近
//    - iou大于MERGE_MIN_IOU
//    - 则看作是同一个目标,合并两者的坐标点
//    5. 最终indices中存储保留下来的检测框索引
//            这样通过nms可以过滤重复的检测框,合并技术同一目标的检测框。
//    nms_threshold控制nms的阈值。
//    MERGE_MIN_IOU控制合并的敏感度。

    OpenVINODetector :: OpenVINODetector (
            const std::string model_path,
            const std::string & device_name,
            float conf_threshold, int top_k, float nms_threshold, bool auto_init):
            model_path_(model_path),
            device_name_(device_name),
            conf_threshold_(conf_threshold),
            top_k_(top_k),
            nms_threshold_(nms_threshold) {

        if (auto_init) {
            init();
        }
    }

    void OpenVINODetector :: init() {
        //创建ov_core核心
        if (ov_core_ == nullptr) {
            ov_core_ = std::make_unique<ov::Core>();
        }

        //  // 读取模型
        auto model = ov_core_->read_model(model_path_);

        //// 设置模型推理类型为浮点
        //  ov::preprocess::PrePostProcessor ppp(model);
        //  ppp.input().tensor().set_element_type(ov::element::f32);
        //  ppp.output().tensor().set_element_type(ov::element::f32);
        // Set infer type
        ov::preprocess::PrePostProcessor ppp(model);
        // Set input output precision
        ppp.input().tensor().set_element_type(ov::element::f32);
        ppp.output().tensor().set_element_type(ov::element::f32);

        // Compile model
        // 编译模型
        compiled_model_ = std::make_unique<ov::CompiledModel>(
                ov_core_->compile_model(
                        model, device_name_,
                        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)));
        // 设置特征图步长
        strides_ = {8, 16, 32};
        // 根据输入大小和步长生成特征图格网信息
        generate_grids_and_stride(INPUT_W, INPUT_H, strides_, grid_strides_);
    }

    std::future<bool> OpenVINODetector :: push_input(const cv::Mat & rgb_img, int64_t timestamp_nanosec)
    {
        if (rgb_img.empty()) {
            // return false when image is empty
            return std::async([]() {return false;});
        }

        // Reprocess
        // 图像缩放 letterbox  获取缩放矩阵
        Eigen::Matrix3f transform_matrix;  // transform matrix from resized image to source image.
        cv::Mat resized_img = letterbox(rgb_img, transform_matrix);

        // Start async detect
        // 异步启动检测线程
        return std::async(
                std::launch::async, &OpenVINODetector::process_callback, this, resized_img, transform_matrix,
                timestamp_nanosec, rgb_img);
        //返回一个future对象,检测完成后会设置结果

    }

    //完成了模型推理及后处理生成最终检测框
    bool OpenVINODetector :: process_callback(const cv::Mat resized_img, Eigen::Matrix3f transform_matrix,
            int64_t timestamp_nanosec, const cv::Mat & src_img) {

        // BGR->RGB, u8(0-255)->f32(0.0-1.0), HWC->NCHW
        // note: TUP's model no need to normalize
        // 图像预处理
        cv::Mat blob =
                cv::dnn::blobFromImage(
                        resized_img, 1., cv::Size(INPUT_W, INPUT_H), cv::Scalar(0, 0, 0),
                        true);

        // Feed blob into input
        // 设置输入
        auto input_port = compiled_model_->input();
        ov::Tensor input_tensor(input_port.get_element_type(),
                                ov::Shape(std::vector<size_t>{1, 3, INPUT_W, INPUT_H}),
                                blob.ptr(0));

        // Start inference
        // 推理
        auto infer_request = compiled_model_->create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        // 获取输出
        auto output = infer_request.get_output_tensor();

        // Process output data
        // 解析输出
        auto output_shape = output.get_shape();
        // 3549 x 21 Matrix
        cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());

        // Parsed variable
        std::vector<ArmorObject> objs_tmp, objs_result;
        std::vector<cv::Rect> rects;
        std::vector<float> scores;
        std::vector<int> indices;

        // Parse YOLO output
        // 生成proposals候选框
        generate_proposals(
                objs_tmp, scores, rects, output_buffer, transform_matrix,
                this->conf_threshold_, this->grid_strides_);

        // TopK
        // 排序取topk 即候选框个数
        std::sort(objs_tmp.begin(), objs_tmp.end(),
                  [](const ArmorObject & a, const ArmorObject & b) {
                    return a.prob > b.prob;
                });

        if (objs_tmp.size() > static_cast<size_t>(this->top_k_)) {
            objs_tmp.resize(this->top_k_);
        }

        // NMS去重
        nms_merge_sorted_bboxes(objs_tmp, indices, this->nms_threshold_);
        // 处理合并后框
        for (size_t i = 0; i < indices.size(); i++) {
            objs_result.push_back(std::move(objs_tmp[indices[i]]));

            if (objs_result[i].pts.size() >= 8) {
                auto N = objs_result[i].pts.size();
                cv::Point2f pts_final[4];

                for (size_t j = 0; j < N; j++) {
                    pts_final[j % 4] += objs_result[i].pts[j];
                }

                objs_result[i].pts.resize(4);
                for (int j = 0; j < 4; j++) {
                    pts_final[j].x /= static_cast<float>(N) / 4.0;
                    pts_final[j].y /= static_cast<float>(N) / 4.0;
                    // 保存结果
                    objs_result[i].pts[j] = pts_final[j];
                }
            }
        }
        m_objs = objs_result;
        return false;
    }

    std::vector<ArmorObject> vgd_detect::OpenVINODetector::getObjects(){
        return m_objs;
    }
//1. 返回类内成员变量m_objs,其中存储了   每次检测得到的检测框结果
//2. m_objs在process_callback函数中被更新,存储了经过NMS等后处理后的最终检测框
//3. 通过该接口,外部调用可以获取到每次检测的结果
//4. std::vector用于存储多个检测框,每个框用ArmorObject类表示
//5. 返回vector容器,提供了迭代访问所有检测框的能力
}  // namespace vgd_detect


//padding在图像处理里指的是对图像边缘进行扩充,以填充更大的区域。
//在这个letterbox函数中,padding的计算是:
//- pad_h = new_shape[1] - resize_h
//- pad_w = new_shape[0] - resize_w
//这里新图像大小new_shape已经确定了,例如416x416。
//resize_h和resize_w是根据原图大小计算的缩放后图像的高和宽。
//那么padding的大小就等于新图像尺寸减去缩放后的图像尺寸。
//也就是需要扩充的大小,来填充从缩放图像到新图像的空余区域。
//这么计算出来的pad_h和pad_w就是上下和左右两侧需要填充的像素大小。
//后面会在图像边界均匀地填充这些大小的padding,将缩放后的图像润满到新图像大小。
//这样经过缩放和padding后的图像就能够达到模型要求的标准尺寸,但又保持了原图的长宽比,不会造成图像失真