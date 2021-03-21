//
// Created by Witold Sempruch on 13.03.2021.
//

#ifndef OBJECTRECONSTRUCTION_POINTCLOUD_H
#define OBJECTRECONSTRUCTION_POINTCLOUD_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "3rdParty/tinyXML/tinyxml2.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <boost/filesystem.hpp>
#include "Helpers.h"

class PointCloud {
public:
    std::string _dataPathDepth;
    std::string _dataPathRGB;
    std::string _savePath;
    std::string _transformsPath;
    Eigen::Matrix3f _cameraMatrix;
    float _farClippingPlane;
    float _maxDepth;
    float _depthScale;
    int _imgWidth;
    int _imgHeight;

    PointCloud(const std::string &dataPathDepth,
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
               const float &c_y);

    pcl::PointCloud<pcl::PointXYZ> pointCloudFromDepth(int index);

    pcl::PointCloud<pcl::PointXYZRGB> pointCloudFromDepthRGB(int index);

    cv::Mat imageFromPointCloud(pcl::PointCloud<pcl::PointXYZ> &inputPCD, int &index);

    void setCameraMatrix(const float &f_x, const float &f_y, const float &c_x, const float &c_y);

    Eigen::Matrix4f getTransformMatrix(int index) const;
};


#endif //OBJECTRECONSTRUCTION_POINTCLOUD_H
