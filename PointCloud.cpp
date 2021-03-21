//
// Created by Witold Sempruch on 13.03.2021.
//

#include "PointCloud.h"


PointCloud::PointCloud(const std::string &dataPathDepth,
                       const std::string &dataPathRGB,
                       const std::string &savePath,
                       const std::string &transformsPath,
                       const float &farClippingPlane,
                       const float &maxDepth,
                       const int &imgWidth,
                       const int &imgHeight,
                       const float &f_x,
                       const float &f_y,
                       const float &c_x,
                       const float &c_y) {
    _dataPathDepth = dataPathDepth;
    _dataPathRGB = dataPathRGB;
    _savePath = savePath;
    _transformsPath = transformsPath;
    _farClippingPlane = farClippingPlane;
    _maxDepth = maxDepth;
    _depthScale = _farClippingPlane / _maxDepth;
    _imgWidth = imgWidth;
    _imgHeight = imgHeight;
    _cameraMatrix = Eigen::Matrix3f::Zero();
    setCameraMatrix(f_x, f_y, c_x, c_y);


}

pcl::PointCloud<pcl::PointXYZ> PointCloud::pointCloudFromDepth(int index) {
    boost::filesystem::path imgName(std::to_string(index) + ".exr");
    boost::filesystem::path imgPath = boost::filesystem::path(_dataPathDepth) / imgName;
    cv::Mat depthImg = cv::imread(imgPath.string(), cv::IMREAD_ANYDEPTH);
    pcl::PointCloud<pcl::PointXYZ> depthCloud;
    Eigen::Vector3f imageCoords;
    Eigen::Vector3f worldCoords;
    pcl::PointXYZ point;
    for (int row = 0; row < depthImg.rows; row++) {
        for (int col = 0; col < depthImg.cols; col++) {
            float d = depthImg.at<float>(row, col) * _depthScale;
            imageCoords << col, row, 1;
            worldCoords = d * _cameraMatrix * imageCoords;
            point.x = worldCoords.x();
            point.y = worldCoords.y();
            point.z = worldCoords.z();
            depthCloud.push_back(point);
        }
    }
    return depthCloud;

}

pcl::PointCloud<pcl::PointXYZRGB> PointCloud::pointCloudFromDepthRGB(int index) {
    boost::filesystem::path depthImgName(std::to_string(index) + ".exr");
    boost::filesystem::path rgbImgName(std::to_string(index));
    boost::filesystem::path depthImgPath = boost::filesystem::path(_dataPathDepth) / depthImgName;
    boost::filesystem::path rgbImgPath = boost::filesystem::path(_dataPathRGB) / rgbImgName;
    cv::Mat depthImg = cv::imread(depthImgPath.string(), cv::IMREAD_ANYDEPTH);
    cv::Mat rgbImg = cv::imread(rgbImgPath.string(), cv::IMREAD_COLOR);
    cv::cvtColor(rgbImg, rgbImg, cv::COLOR_BGR2RGB);
    pcl::PointCloud<pcl::PointXYZRGB> depthCloud;
    Eigen::Vector3f imageCoords;
    Eigen::Vector3f worldCoords;
    pcl::PointXYZRGB point;
    for (int row = 0; row < depthImg.rows; row++) {
        for (int col = 0; col < depthImg.cols; col++) {
            float d = depthImg.at<float>(row, col) * _depthScale;
            cv::Vec3b pixelColor = rgbImg.at<cv::Vec3b>(row, col);
            imageCoords << col, row, 1;
            worldCoords = d * _cameraMatrix * imageCoords;
            point.x = worldCoords.x();
            point.y = worldCoords.y();
            point.z = worldCoords.z();
            point.r = pixelColor[0];
            point.g = pixelColor[1];
            point.b = pixelColor[2];
            depthCloud.push_back(point);
        }
    }
    return depthCloud;
}



cv::Mat PointCloud::imageFromPointCloud(std::vector<cv::Point3f> &points) {
    return cv::Mat();
}


void PointCloud::setCameraMatrix(const float &f_x, const float &f_y, const float &c_x, const float &c_y) {
    _cameraMatrix(0, 0) = 1 / f_x;
    _cameraMatrix(0, 2) = (-c_x) / f_x;
    _cameraMatrix(1, 1) = 1 / f_y;
    _cameraMatrix(1, 2) = (-c_y) / f_y;
    _cameraMatrix(2, 2) = 1;
}

Eigen::Matrix4f PointCloud::getTransformMatrix(int index) const {
    try {
        tinyxml2::XMLDocument transformsFile;
        transformsFile.LoadFile(_transformsPath.c_str());
        if (transformsFile.ErrorID())
            std::cout << "Unable to load config file" << std::endl;
        tinyxml2::XMLElement *file = transformsFile.FirstChildElement("Files")->FirstChildElement("File");
        while (!file->Attribute("Index", std::to_string(index).c_str())) {
            file = file->NextSiblingElement();
        }
        std::string matRow0 = file->FirstChildElement("MatrixRow0")->GetText();
        std::string matRow1 = file->FirstChildElement("MatrixRow1")->GetText();
        std::string matRow2 = file->FirstChildElement("MatrixRow2")->GetText();
        std::string matRow3 = file->FirstChildElement("MatrixRow3")->GetText();
        std::vector<std::vector<float>> rows = {Helpers::processMatrixRow(matRow0),
                                                Helpers::processMatrixRow(matRow1),
                                                Helpers::processMatrixRow(matRow2),
                                                Helpers::processMatrixRow(matRow3)};

        Eigen::Matrix4f transformMatrix = Helpers::matrixFromRows(rows);
        return transformMatrix;
    }
    catch (const std::exception &ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return Eigen::Matrix4f::Identity();
    }


}





