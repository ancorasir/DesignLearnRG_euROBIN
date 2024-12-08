#ifndef TASKBOARD_DETECTOR_H
#define TASKBOARD_DETECTOR_H

#include <memory>
#include <opencv2/core.hpp>

// In our solution, we depend on
// high-precision taskboard localization

// Taskboard Localization (In Camera)

// Step 1 : Use YOLO detector To Find Position of Buttons and Hole

// Step 2 : Registrate these points to CAD model and Find Frame of Taskboard
/**
 * @brief The Taskboard(tb) Frame
 *
 * Z |    / Y
 *   |   /
 *   |  /
 *   | /
 *   |/
 *   o--------------
 *   ^              X
 * Blue-Button
 */

class YOLO_V8;

class TaskboardDetector
{
public:
    TaskboardDetector(const std::string model_path /*= RESOURCE_PATH_STR"best.onnx"*/,
                      const std::string class_yaml_path /*= RESOURCE_PATH_STR"taskboard.yaml"*/);

    void detect(const cv::Mat &input,
                cv::Mat &output,
                std::vector<cv::Point2d> &observed_points);

private:
    void readClassYaml();

    std::string modelPath_;

    std::string classYamlPath_;

    std::shared_ptr<YOLO_V8> yoloDetector_;
};

#endif // TASKBOARD_DETECTOR_H
