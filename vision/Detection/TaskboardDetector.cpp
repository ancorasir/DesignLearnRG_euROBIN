#include "TaskboardDetector.h"
#include "inference.h"
#include <fstream>

using namespace cv;

TaskboardDetector::TaskboardDetector(const std::string model_path, const std::string class_yaml_path) : modelPath_(model_path),
                                                                                                        classYamlPath_(class_yaml_path),
                                                                                                        yoloDetector_(new YOLO_V8)
{
    readClassYaml();

    DL_INIT_PARAM params;
    params.rectConfidenceThreshold = 0.2;
    params.iouThreshold = 0.5;
    params.modelPath = modelPath_;
    params.imgSize = {640, 640};

#ifdef USE_CUDA
    params.cudaEnable = true;

    // GPU FP32 inference
    params.modelType = YOLO_DETECT_V8;
    // GPU FP16 inference
    // Note: change fp16 onnx model
    // params.modelType = YOLO_DETECT_V8_HALF;

#else
    // CPU inference
    params.modelType = YOLO_DETECT_V8;
    params.cudaEnable = false;

#endif

    yoloDetector_->CreateSession(params);
}

// Each Detection Cost about 70ms on CPU
// Input Image
// Output Annotated Image
void TaskboardDetector::detect(const cv::Mat &input,
                               cv::Mat &output,
                               std::vector<Point2d> &observed_points)
{
    observed_points.resize(3);
    input.copyTo(output);

    std::vector<DL_RESULT> res;
    yoloDetector_->RunSession(output, res);

    for (auto &re : res)
    {
        int result_id = re.classId;

        if (result_id == 0)
        {
            cv::Rect bounding_box = re.box;
            float cx = bounding_box.x + bounding_box.width / 2.0;
            float cy = bounding_box.y + bounding_box.height / 2.0;
            Point2d center(cx, cy);
            observed_points[0] = center;

            cv::RNG rng(cv::getTickCount());
            cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            float radius = fmin(bounding_box.width, bounding_box.height) / 2.0;
            cv::circle(output, cv::Point((int)cx, (int)cy), (int)radius, color, 3);

            float confidence = floor(100 * re.confidence) / 100;
            std::cout << std::fixed << std::setprecision(2);
            std::string label = "BlueBtn" +
                                std::to_string(confidence).substr(0, std::to_string(confidence).size() - 4);

            cv::putText(
                output,
                label,
                cv::Point(re.box.x, re.box.y - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.75,
                cv::Scalar(144, 238, 144),
                2);
        }

        if (result_id == 1)
        {
            cv::Rect bounding_box = re.box;
            float cx = bounding_box.x + bounding_box.width / 2.0;
            float cy = bounding_box.y + bounding_box.height / 2.0;

            cv::RNG rng(cv::getTickCount());
            cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            float radius = fmin(bounding_box.width, bounding_box.height) / 2.0;
            cv::circle(output, cv::Point((int)cx, (int)cy), (int)radius, color, 3);
        }

        if (result_id == 3)
        {
            cv::Rect bounding_box = re.box;
            float cx = bounding_box.x + bounding_box.width / 2.0;
            float cy = bounding_box.y + bounding_box.height / 2.0;
            Point2d center(cx, cy);
            observed_points[1] = center;

            cv::RNG rng(cv::getTickCount());
            cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            float radius = fmin(bounding_box.width, bounding_box.height) / 2.0;
            cv::circle(output, cv::Point((int)cx, (int)cy), (int)radius, color, 3);

            float confidence = floor(100 * re.confidence) / 100;
            std::cout << std::fixed << std::setprecision(2);
            std::string label = "RedBtn" +
                                std::to_string(confidence).substr(0, std::to_string(confidence).size() - 4);

            cv::putText(
                output,
                label,
                cv::Point(re.box.x, re.box.y + 70),
                cv::FONT_HERSHEY_SIMPLEX,
                0.75,
                cv::Scalar(144, 238, 144),
                2);
        }

        if (result_id == 4)
        {
            cv::Rect bounding_box = re.box;
            float cx = bounding_box.x + bounding_box.width / 2.0;
            float cy = bounding_box.y + bounding_box.height / 2.0;
            Point2d center(cx, cy);
            observed_points[2] = center;

            cv::RNG rng(cv::getTickCount());
            cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
            float radius = fmin(bounding_box.width, bounding_box.height) / 2.0;
            cv::circle(output, cv::Point((int)cx, (int)cy), (int)radius, color, 3);

            float confidence = floor(100 * re.confidence) / 100;
            std::cout << std::fixed << std::setprecision(2);
            std::string label = "RedHole" +
                                std::to_string(confidence).substr(0, std::to_string(confidence).size() - 4);

            cv::putText(
                output,
                label,
                cv::Point(re.box.x, re.box.y - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.75,
                cv::Scalar(144, 238, 144),
                2);
        }
    }
}

void TaskboardDetector::readClassYaml()
{
    // Open the YAML file
    std::ifstream file(classYamlPath_);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file" << std::endl;
        return;
    }

    // Read the file line by line
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line))
    {
        lines.push_back(line);
    }

    // Find the start and end of the names section
    std::size_t start = 0;
    std::size_t end = 0;
    for (std::size_t i = 0; i < lines.size(); i++)
    {
        if (lines[i].find("names:") != std::string::npos)
        {
            start = i + 1;
        }
        else if (start > 0 && lines[i].find(':') == std::string::npos)
        {
            end = i;
            break;
        }
    }

    // Extract the names
    std::vector<std::string> names;
    for (std::size_t i = start; i < end; i++)
    {
        std::stringstream ss(lines[i]);
        std::string name;
        std::getline(ss, name, ':'); // Extract the number before the delimiter
        std::getline(ss, name);      // Extract the string after the delimiter
        names.push_back(name);
    }

    yoloDetector_->classes = names;
}
