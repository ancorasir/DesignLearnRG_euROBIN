#ifndef DATARELAY_H
#define DATARELAY_H

#include <iostream>
#include <thread>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include "robot.pb.h"

struct RobotData
{
    RobotData() {}

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    // Pose order (x,y,z,rx,ry,rz)
    std::vector<double> tcp_pose_;
    std::vector<double> camera_pose_;

    Eigen::Affine3d eMc;
    Eigen::Affine3d cMw;

    void getHomogeneousMatrix()
    {
        Eigen::Vector3d t_vec(tcp_pose_.at(0), tcp_pose_.at(1), tcp_pose_.at(2));
        Eigen::Vector3d r_vec(tcp_pose_.at(3), tcp_pose_.at(4), tcp_pose_.at(5));

        Eigen::Affine3d wMe, wMc, eMw;
        wMe.setIdentity();
        wMe.translate(t_vec).rotate(Eigen::AngleAxisd(r_vec.norm(), r_vec.normalized()));

        wMc = wMe * eMc;
        Eigen::Vector3d eigen_t_vec(wMc.translation());
        Eigen::AngleAxisd eigen_r_vec(wMc.rotation());

        camera_pose_.clear();
        camera_pose_.push_back(eigen_t_vec.x());
        camera_pose_.push_back(eigen_t_vec.y());
        camera_pose_.push_back(eigen_t_vec.z());
        camera_pose_.push_back(eigen_r_vec.angle()*eigen_r_vec.axis().x());
        camera_pose_.push_back(eigen_r_vec.angle()*eigen_r_vec.axis().y());
        camera_pose_.push_back(eigen_r_vec.angle()*eigen_r_vec.axis().z());

        cMw = wMc.inverse();
    }

    void updateData(const RobotData& tmp_data)
    {
        joint_positions_ = tmp_data.joint_positions_;
        joint_velocities_ = tmp_data.joint_velocities_;
        tcp_pose_ = tmp_data.tcp_pose_;
        camera_pose_ = tmp_data.camera_pose_;
        eMc = tmp_data.eMc;
        cMw = tmp_data.cMw;
    }


};

enum Detection
{
    TaskBoard = 0,
    Triangle = 1,
};

class DataRelayApp {
public:
    DataRelayApp();

    void run();

private:
    zmq::context_t context;
    zmq::socket_t publisher;
    zmq::socket_t requestServer;

    RobotData robotData;
    cv::Mat imageData;
    std::mutex mtx;
    std::vector<cv::Point2d> keypoints;
    std::vector<cv::Point2d> bb_point_collections;
    std::vector<cv::Point2d> rb_point_collections;
    std::vector<cv::Point2d> rh_point_collections;
    Eigen::Affine3d eMc;

    // For TASKBOARD
    const int numSamples = 20; // Number of samples to average
    int sampleCounter = 0; // Counter to track collected samples
    bool collecting = false; // Flag to indicate if we are currently collecting samples
    Detection detectMode = Detection::TaskBoard;

    // For Slider Triangle
    std::vector<int> pixel_distance_collections;
    const int numAverage = 5;
    int pixelDistance = 0;

    void captureImages();
    void getRobotData();
    void publishData();
    void handleRequests();
    std::string serializeImage(const cv::Mat& image);
    void loadExtrinsic();
    robot::Robot convertToRobotMessage(const RobotData& robot, const cv::Mat& img);
    void calculateAvgPixelCord(const std::vector<cv::Point2d>& imagePoints,
                               cv::Point2d& avgImagePoint);
    int mostFrequent(const std::vector<int>& nums);

    std::string intrinsic_file {RESOURCE_PATH_STR"ur_camera.yaml"};
    std::string robot_ip = "192.168.1.102";
    std::string model_path {RESOURCE_PATH_STR"best_taskboard.onnx"};
    std::string model_path_sceen {RESOURCE_PATH_STR"best_screen.onnx"};
    std::string class_yaml_path{RESOURCE_PATH_STR"taskboard.yaml"};
    std::string extrinsic_file {RESOURCE_PATH_STR"eMc.yaml"};
    std::string model_point_file {RESOURCE_PATH_STR"model_data.yaml"};

};
#endif // DATARELAY_H
