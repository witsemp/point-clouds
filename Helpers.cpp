//
// Created by Witold Sempruch on 20.03.2021.
//
#include "Helpers.h"

std::vector<float> Helpers::processMatrixRow(std::string row) {
    std::vector<float> result;
    row.erase(std::remove(row.begin(), row.end(), ','), row.end());
    row.erase(std::remove(row.begin(), row.end(), '('), row.end());
    row.erase(std::remove(row.begin(), row.end(), ')'), row.end());
    std::istringstream iss(row);
    for (std::string s; iss >> s;)
        result.push_back(std::stof(s));
    return result;
}

Eigen::Matrix4f Helpers::matrixFromRows(std::vector<std::vector<float>> rows) {
    Eigen::Matrix4f mat;
    for (int i = 0; i < mat.rows(); i++) {
        mat.row(i) << rows[i][0], rows[i][1], rows[i][2], rows[i][3];
    }
    return mat;
}

pcl::PointXYZ Helpers::pclFromEigen(Eigen::Vector3f vec) {
    pcl::PointXYZ pointPCL;
    pointPCL.x = vec.x();
    pointPCL.y = vec.y();
    pointPCL.z = vec.z();
    return pointPCL;
}

std::string Helpers::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }
    r += "C";
    r += (chans + '0');
    return r;
}


