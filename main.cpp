#include <iostream>
#include "PointCloud.h"

#include <pcl/visualization/cloud_viewer.h>
#include <boost/lambda/bind.hpp>


int main() {
    using namespace boost::filesystem;
    using namespace boost::lambda;

    const std::string DATA_PATH_DEPTH("/home/witsemp/work/31032021/Multimodel4/chair/1a74a83fa6d24b3cacd67ce2c72c02e/depth");
    const std::string DATA_PATH_RGB("/home/witsemp/work/31032021/Multimodel4/chair/1a74a83fa6d24b3cacd67ce2c72c02e/rgb");
    const std::string SAVE_PATH("/home/witsemp/work/31032021/Multimodel3_Projected");
    const std::string TRANSFORMS_PATH("/home/witsemp/work/31032021/Multimodel4.xml");
    const float FAR_CLIPPING_PLANE = 10.0;
    const float MAX_DEPTH = 1.09;
    const int IMG_WIDTH = 640;
    const int IMG_HEIGHT = 480;
    const float F_X = 525.0;
    const float F_Y = 525.0;
    const float C_X = 320;
    const float C_Y = 240;
    int idx = 4;
    PointCloud pcd(DATA_PATH_DEPTH, DATA_PATH_RGB, SAVE_PATH, TRANSFORMS_PATH, FAR_CLIPPING_PLANE, MAX_DEPTH, IMG_WIDTH,
                   IMG_HEIGHT, F_X,
                   F_Y, C_X, C_Y);

    std::string depthPath("/home/witsemp/work/31032021/Multimodel4/chair/1a74a83fa6d24b3cacd67ce2c72c02e/depth/1a74a83fa6d24b3cacd67ce2c72c02e_82.exr");
    std::string rgbPath("/home/witsemp/work/31032021/Multimodel4/chair/1a74a83fa6d24b3cacd67ce2c72c02e/rgb/1a74a83fa6d24b3cacd67ce2c72c02e_82");
    path p_depthPath = path(depthPath);
    path p_rgbPath = path(rgbPath);
    pcl::PointCloud<pcl::PointXYZRGB> p = pcd.pointCloudFromDepthRGB(p_depthPath, p_rgbPath);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&p);
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (ptrCloud);
    while (!viewer.wasStopped ())
    {
    }




//    path dataPath(DATA_PATH_DEPTH);
//    std::vector<directory_entry> directories;
//    for (auto &classEntry : boost::make_iterator_range(directory_iterator(dataPath), {})) {
//        if (is_directory(classEntry.path())) {
//            path classPath = dataPath / classEntry.path().filename();
//            std::cout << "Processing class: " << classEntry.path().string() << std::endl;
////            pcd._transformsPath = classEntry.path().string() + ".xml";
//            std::cout << pcd._transformsPath << std::endl;
//            for (auto &modelEntry : boost::make_iterator_range(directory_iterator(classPath), {})) {
//                path modelPath = classPath / modelEntry.path().filename() / path("depth");
//                std::cout << "Processing model: " << modelEntry.path().string() << std::endl;
//                for (auto &imageEntry : boost::make_iterator_range(directory_iterator(modelPath), {})) {
//                    std::string fileName = imageEntry.path().stem().string();
//                    int index = std::stoi(fileName.substr(fileName.find("_") + 1, 5));
//                    if ((index % 2 == 0)) {
//                        path imagePath = imageEntry.path();
//                        path imageSavePath = path(SAVE_PATH) / imageEntry.path().filename();
//                        pcl::PointCloud<pcl::PointXYZ> p = pcd.pointCloudFromDepth(imagePath);
//                        pcd.imageFromPointCloud(p, index, imageSavePath);
//                    }
//                }
//            }
//        }
//    }

//    path dataPath(DATA_PATH_DEPTH);
//    int dataCnt = std::count_if(
//            directory_iterator(dataPath),
//            directory_iterator(),
//            static_cast<bool(*)(const path&)>(is_regular_file) );
//    for (int index = 0; index < dataCnt; index += 2)
//    {
//        std::cout << "Processing image " << index << " out of " << dataCnt << std::endl;
//        pcl::PointCloud<pcl::PointXYZ> basePCD = pcd.pointCloudFromDepth(index);
//        pcd.imageFromPointCloud(basePCD, index);
//    }



}
