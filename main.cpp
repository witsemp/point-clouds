#include <iostream>
#include "PointCloud.h"

#include <pcl/visualization/cloud_viewer.h>


int main() {
    const std::string DATA_PATH_DEPTH("/home/witsemp/work/24032021/toy_data/chair/1a74a83fa6d24b3cacd67ce2c72c02e/depth");
    const std::string DATA_PATH_RGB("/home/witsemp/work/24032021/toy_data/chair/1a74a83fa6d24b3cacd67ce2c72c02e/rgb");
    const std::string SAVE_PATH("/home/witsemp/work/24032021/");
    const std::string TRANSFORMS_PATH("/home/witsemp/work/24032021/toy_data_output.xml");
    const float FAR_CLIPPING_PLANE = 10.0;
    const float MAX_DEPTH = 1.09;
    const int IMG_WIDTH = 1280;
    const int IMG_HEIGHT = 960;
    const float F_X = 1062.3;
    const float F_Y = 1062.3;
    const float C_X = 639.5;
    const float C_Y = 479.5;
    int idx = 10;
    PointCloud pcd(DATA_PATH_DEPTH, DATA_PATH_RGB, SAVE_PATH, TRANSFORMS_PATH, FAR_CLIPPING_PLANE, MAX_DEPTH, IMG_WIDTH, IMG_HEIGHT, F_X,
                   F_Y, C_X, C_Y);
    Eigen::Matrix4f mat = pcd.getTransformMatrix(idx);
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud10 = pcd.pointCloudFromDepthRGB(idx);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloudPTR = pointCloud10;
    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(cloudPTR);
    while (!viewer.wasStopped()) {

    }


}
