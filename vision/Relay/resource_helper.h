#pragma once
#include <iostream>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace ResourceHelper
{

    inline void loadExtrinsic(const std::string &extrinsic_file, Eigen::Affine3d &wMc)
    {
        cv::Mat mat;
        // Create a FileStorage object in READ mode
        cv::FileStorage fs(extrinsic_file, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cerr << "Failed to open file for reading: " << extrinsic_file << std::endl;
            return; // Return an empty matrix
        }

        // Read the matrix from the YAML file
        fs["matrix"] >> mat;

        // Release the FileStorage object
        fs.release();

        Eigen::Vector3d t_vec(mat.at<double>(0, 0), mat.at<double>(1, 0), mat.at<double>(2, 0));
        Eigen::Vector3d r_vec(mat.at<double>(3, 0), mat.at<double>(4, 0), mat.at<double>(5, 0));

        wMc.setIdentity();
        wMc.translate(t_vec).rotate(Eigen::AngleAxisd(r_vec.norm(), r_vec.normalized()));
    }

    inline void getHomogeneousMatrix(const std::vector<double> &pose, Eigen::Affine3d &mat)
    {
        Eigen::Vector3d t_vec(pose.at(0), pose.at(1), pose.at(2));
        Eigen::Vector3d r_vec(pose.at(3), pose.at(4), pose.at(5));

        mat.setIdentity();
        mat.translate(t_vec).rotate(Eigen::AngleAxisd(r_vec.norm(), r_vec.normalized()));
    }

}
