#include "DataRelay.h"
#include "TaskboardDetector.h"
#include "TaskboardPoseEstimator.h"
#include "TriangleDetector.h"
#include <librealsense2/rs.hpp>
#include <ur_rtde/rtde_receive_interface.h>
#include "resource_helper.h"
#include "detection.pb.h"

DataRelayApp::DataRelayApp()
    : context(1),
    publisher(context, ZMQ_PUB),
    requestServer(context, ZMQ_REP) {

    publisher.bind("tcp://*:5555");
    requestServer.bind("tcp://*:5556");

    loadExtrinsic();
    robotData.eMc = eMc;
}

void DataRelayApp::run() {
    std::thread imageThread(&DataRelayApp::captureImages, this);
    std::thread robotThread(&DataRelayApp::getRobotData, this);
    std::thread publisherThread(&DataRelayApp::publishData, this);
    std::thread requestThread(&DataRelayApp::handleRequests, this);

    imageThread.join();
    robotThread.join();
    publisherThread.join();
    requestThread.join();
}

void DataRelayApp::captureImages() {

    rs2::pipeline pipeline;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile profile = pipeline.start(cfg);

    TaskboardDetector taskboard_detector(model_path, class_yaml_path);
    TriangleDetector triangle_detector(model_path_sceen);

    while (true) {

        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::frame colorFrame = frames.get_color_frame();
        // Convert to cv::Mat
        const int w = colorFrame.as<rs2::video_frame>().get_width();
        const int h = colorFrame.as<rs2::video_frame>().get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

        Detection mode;
        {
            std::lock_guard<std::mutex> lock(mtx);
            mode = detectMode;
        }

        if(mode == Detection::TaskBoard)
        {
            // Detect keypoints
            cv::Mat imageOut;
            std::vector<cv::Point2d> kp;
            taskboard_detector.detect(image, imageOut, kp);
            {
                std::lock_guard<std::mutex> lock(mtx);
                imageData = imageOut.clone();
                keypoints = kp;
            }
        }

        if(mode == Detection::Triangle)
        {
            cv::Mat imageOut;
            int pixel_dis;
            triangle_detector.detect(image, imageOut, pixel_dis);
            {
                std::lock_guard<std::mutex> lock(mtx);
                imageData = imageOut.clone();
                pixelDistance = pixel_dis;
            }
        }

        // You can add code to update robotData here if needed
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust as needed
    }
}

void DataRelayApp::getRobotData() {
    // Initialize UR RTDE and continuously retrieve robot data
    // Update robotData with current robot state
    ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip);
    while (true) {

        RobotData tmp_data;

        // For demonstration purposes; update with real data
        tmp_data.eMc = eMc;
        tmp_data.joint_positions_= rtde_receive.getActualQ();
        tmp_data.tcp_pose_ = rtde_receive.getActualTCPPose();
        tmp_data.joint_velocities_ = rtde_receive.getActualQd();
        tmp_data.getHomogeneousMatrix();

        {
            std::lock_guard<std::mutex> lock(mtx);
            robotData.updateData(tmp_data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust as needed
    }
}

void DataRelayApp::publishData() {

    std::this_thread::sleep_for(std::chrono::seconds(2));

    while (true) {

        RobotData tmp_data;
        cv::Mat tmp_img;

        {
            std::lock_guard<std::mutex> lock(mtx);
            tmp_data.updateData(robotData);
            tmp_img = imageData.clone();
        }

        // Convert and publish the data
        robot::Robot robotMsg = convertToRobotMessage(tmp_data,tmp_img);
        zmq::message_t message(robotMsg.ByteSize());
        robotMsg.SerializeToArray(message.data(), message.size());
        publisher.send(message, zmq::send_flags::none);

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust as needed
    }
}

void DataRelayApp::handleRequests() {

    TaskboardPoseEstimator estimator(intrinsic_file, model_point_file);

    while (true) {
        zmq::message_t request;
        requestServer.recv(request, zmq::recv_flags::none);

        if(request.to_string() == "pose")
        {
            // Start collecting keypoints if not already collecting
            sampleCounter = 0; // Reset sample counter
            bb_point_collections.clear();    // Clear the collected keypoints
            rb_point_collections.clear();
            rh_point_collections.clear();
            std::vector<double> calculated_pose(6,0.);

            // Collect keypoints for the specified number of samples
            while (sampleCounter < numSamples) {
                // Wait for the next frame to be ready
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust delay as necessary

                // Lock the mutex to access the keypoints
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (!keypoints.empty()) {
                        // Store a copy of the current keypoints
                        bb_point_collections.push_back(keypoints.at(0));
                        rb_point_collections.push_back(keypoints.at(1));
                        rh_point_collections.push_back(keypoints.at(2));
                    }
                }

                sampleCounter++;
                std::cerr << "Collected keypoints set #" << sampleCounter << "\n";
            }

            // After collecting enough samples, calculate the average
            if (sampleCounter >= numSamples) {
                // Calculate average keypoints
                // Now compute the average keypoints (placeholder logic)
                std::vector<cv::Point2d> averaged_keypoints;
                averaged_keypoints.resize(3);
                calculateAvgPixelCord(bb_point_collections, averaged_keypoints.at(0));
                calculateAvgPixelCord(rb_point_collections, averaged_keypoints.at(1));
                calculateAvgPixelCord(rh_point_collections, averaged_keypoints.at(2));

                Eigen::Affine3d cMw;
                double init_x,init_y;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cMw = robotData.cMw;
                    init_x = robotData.tcp_pose_.at(0);
                    init_y = robotData.tcp_pose_.at(1);
                }
                // Compute the pose using the averaged keypoints
                estimator.addFeaturePoints(averaged_keypoints, cMw);

                // Workpiece reference with respect to Robot Base
                double estimated_wMo_parameter[3] = {init_x,
                                                     init_y,
                                                     0.};

                estimator.estimatePose(estimated_wMo_parameter);

                // (x,y,z,rx,ry,rz)
                calculated_pose[0] = estimated_wMo_parameter[0];  // x
                calculated_pose[1] = estimated_wMo_parameter[1];  // y
                calculated_pose[2] = 0.100;  // z
                calculated_pose[3] = 0;  // rx
                calculated_pose[4] = 0;  // ry
                calculated_pose[5] = estimated_wMo_parameter[2];  // rz
            }
            // Assuming some logic to determine the direction

            bool success = true;
            int pixel_dist = 0;

            // Create the response message
            detection::Vision res;

            for (double value : calculated_pose) {
                res.add_pose(value);  // Add each pose value to the response
            }
            res.set_direction(pixel_dist);

            // Serialize the response message
            std::string response_string;
            if (!res.SerializeToString(&response_string)) {
                std::cerr << "Failed to serialize response." << std::endl;
                success = false;
            }

            res.set_success(success);

            // Send the reply back to the requester
            zmq::message_t reply(response_string.data(), response_string.size());
            requestServer.send(reply, zmq::send_flags::none);
        }

        if(request.to_string() == "direction")
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                detectMode = Detection::Triangle;
            }

            // Start collecting keypoints if not already collecting
            sampleCounter = 0; // Reset sample counter
            pixel_distance_collections.clear();
            int pixel_dist = 0;
            std::vector<double> calculated_pose(6,0.);

            // Collect keypoints for the specified number of samples
            while (sampleCounter < numAverage) {
                // Wait for the next frame to be ready
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust delay as necessary

                // Lock the mutex to access the keypoints
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    pixel_distance_collections.push_back(pixelDistance);
                }
                sampleCounter++;
                std::cerr << "Collected pixel distance set #" << sampleCounter << "\n";
            }

            // After collecting enough samples, calculate the average
            if (sampleCounter >= numAverage) {
                // Calculate average pixel distance
                pixel_dist = mostFrequent(pixel_distance_collections);

            }
            // Assuming some logic to determine the direction

            bool success = true;

            // Create the response message
            detection::Vision res;

            for (double value : calculated_pose) {
                res.add_pose(value);  // Add each pose value to the response
            }

            res.set_direction(pixel_dist);

            // Serialize the response message
            std::string response_string;
            if (!res.SerializeToString(&response_string)) {
                std::cerr << "Failed to serialize response." << std::endl;
                success = false;
            }

            res.set_success(success);

            // Send the reply back to the requester
            zmq::message_t reply(response_string.data(), response_string.size());
            requestServer.send(reply, zmq::send_flags::none);

            {
                std::lock_guard<std::mutex> lock(mtx);
                detectMode = Detection::TaskBoard;
            }

        }
    }
}

std::string DataRelayApp::serializeImage(const cv::Mat& image) {
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);
    return std::string(buf.begin(), buf.end());
}

void DataRelayApp::loadExtrinsic()
{
    cv::Mat mat;
    // Create a FileStorage object in READ mode
    cv::FileStorage fs(extrinsic_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for reading: " << extrinsic_file << std::endl;
        return; // Return an empty matrix
    }

    // Read the matrix from the YAML file
    fs["matrix"] >> mat;

    // Release the FileStorage object
    fs.release();

    Eigen::Vector3d t_vec(mat.at<double>(0,0),mat.at<double>(1,0),mat.at<double>(2,0));
    Eigen::Vector3d r_vec(mat.at<double>(3,0),mat.at<double>(4,0),mat.at<double>(5,0));

    eMc.setIdentity();
    eMc.translate(t_vec).rotate(Eigen::AngleAxisd(r_vec.norm(), r_vec.normalized()));
}

robot::Robot DataRelayApp::convertToRobotMessage(const RobotData &robot, const cv::Mat &img)
{
    robot::Robot msg;

    // Add joint positions, velocities, and TCP pose
    for (const auto& angle : robot.joint_positions_) {
        msg.add_joint_angles(angle);
    }

    for (const auto& velocity : robot.joint_velocities_) {
        msg.add_joint_velocities(velocity);
    }

    for (const auto& pose : robot.tcp_pose_) {
        msg.add_tcp_pose(pose);
    }

    for (const auto& pose : robot.camera_pose_) {
        msg.add_color_extrinsics(pose);
    }

    // Serialize the image
    msg.set_color_image(serializeImage(img));

    return msg;
}

void DataRelayApp::calculateAvgPixelCord(const std::vector<cv::Point2d> &imagePoints, cv::Point2d &avgImagePoint)
{
    if(!imagePoints.size())
    {
        std::cerr<<"detected center list is null!"<<std::endl;
        return ;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;

    for(const auto& point : imagePoints) {
        sum_x += point.x;
        sum_y += point.y;
    }

    double avg_x = sum_x / imagePoints.size();
    double avg_y = sum_y / imagePoints.size();

    avgImagePoint = cv::Point2d(avg_x, avg_y);

}



int DataRelayApp::mostFrequent(const std::vector<int>& nums) {
    std::unordered_map<int, int> countMap;

    // Count frequencies of each number
    for (int num : nums) {
        countMap[num]++;
    }

    // Find the number with the maximum frequency
    auto maxFreqPair = std::max_element(countMap.begin(), countMap.end(),
                                        [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                                            return a.second < b.second;
                                        });

    return maxFreqPair->first;
}
