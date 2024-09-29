#ifndef TRIANGLEDETECTOR_H
#define TRIANGLEDETECTOR_H

#include <memory>
#include <opencv2/core.hpp>

class YOLO_V8;

class TriangleDetector
{
public:
    TriangleDetector(const std::string model_path);


    void detect(const cv::Mat &input,
                cv::Mat &output ,
                int& pixel_distance);
private:
    int computePixelDistance(const cv::Mat& src , const cv::Rect &roi);
    std::string modelPath_;
    std::shared_ptr<YOLO_V8> yoloDetector_;
};

#endif // TRIANGLEDETECTOR_H
