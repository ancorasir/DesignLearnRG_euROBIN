#include "TriangleDetector.h"
#include "inference.h"

using namespace std;
using namespace cv;

TriangleDetector::TriangleDetector(const std::string model_path)
    : modelPath_(model_path),
      yoloDetector_(new YOLO_V8)
{
    yoloDetector_->classes = {"screen"};

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

void TriangleDetector::detect(const cv::Mat &input,
                              cv::Mat &output,
                              int &pixel_distance)
{
    input.copyTo(output);
    std::vector<DL_RESULT> res;
    yoloDetector_->RunSession(output, res);

    for (auto &re : res)
    {
        cv::RNG rng(cv::getTickCount());
        cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

        cv::rectangle(output, re.box, color, 3);

        float confidence = floor(100 * re.confidence) / 100;
        std::cout << std::fixed << std::setprecision(2);
        std::string label = "Screen" +
                            std::to_string(confidence).substr(0, std::to_string(confidence).size() - 4);

        cv::putText(
            output,
            label,
            cv::Point(re.box.x, re.box.y - 5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.75,
            cv::Scalar(0, 0, 0),
            2);

        pixel_distance = computePixelDistance(input, re.box);
    }
}

int TriangleDetector::computePixelDistance(const cv::Mat &src, const cv::Rect &roi)
{
    // Create a mask for the ROI
    cv::Mat roiMask = cv::Mat::zeros(src.size(), CV_8UC1);
    roiMask(roi).setTo(cv::Scalar(255)); // Set the ROI to white

    // Convert the image to HSV color space
    cv::Mat hsv;
    cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    // Create a mask for the ROI
    cv::Mat hsvROI;
    src.copyTo(hsvROI, roiMask); // Copy only the ROI area
    hsvROI = hsv(roi);           // Get the HSV of the ROI area

    // Define color ranges for red, green, and yellow
    // Red (two ranges)
    cv::Scalar lowerRed1(0, 100, 100);
    cv::Scalar upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(160, 100, 100);
    cv::Scalar upperRed2(180, 255, 255);

    // Green
    cv::Scalar lowerGreen(40, 100, 100);
    cv::Scalar upperGreen(80, 255, 255);

    // Create masks for each color
    cv::Mat maskRed1, maskRed2, maskGreen, maskRed;
    inRange(hsvROI, lowerRed1, upperRed1, maskRed1);
    inRange(hsvROI, lowerRed2, upperRed2, maskRed2);
    inRange(hsvROI, lowerGreen, upperGreen, maskGreen);
    maskRed = maskRed1 | maskRed2;

    // Distance between Red and Green Triangle in Pixel
    int u_green = 0;
    int u_red = 0;

    {
        // Find contours in the mask
        vector<vector<cv::Point>> contoursGreen;
        findContours(maskGreen, contoursGreen, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contoursGreen.size())
        {
            // Sort contours based on area
            sort(contoursGreen.begin(), contoursGreen.end(), [](const vector<Point> &a, const vector<Point> &b)
                 { return contourArea(a) > contourArea(b); });

            Moments m = moments(contoursGreen.front());
            if (m.m00 != 0)
            {                                                   // Avoid division by zero
                Point2f centroid(m.m10 / m.m00, m.m01 / m.m00); // Calculate centroid
                u_green = centroid.x;
            }
        }
    }

    {
        // Find contours in the mask
        vector<vector<Point>> contoursRed;
        findContours(maskRed, contoursRed, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contoursRed.size())
        {
            // Sort contours based on area
            sort(contoursRed.begin(), contoursRed.end(), [](const vector<Point> &a, const vector<Point> &b)
                 { return contourArea(a) > contourArea(b); });

            Moments m = moments(contoursRed.front());
            if (m.m00 != 0)
            {                                                   // Avoid division by zero
                Point2f centroid(m.m10 / m.m00, m.m01 / m.m00); // Calculate centroid
                u_red = centroid.x;
            }
        }
    }

    return u_green - u_red;
}
