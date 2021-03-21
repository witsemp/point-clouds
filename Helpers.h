//
// Created by Witold Sempruch on 20.03.2021.
//

#ifndef OBJECTRECONSTRUCTION_HELPERS_H
#define OBJECTRECONSTRUCTION_HELPERS_H

#include "PointCloud.h"

namespace Helpers {

    std::vector<float> processMatrixRow(std::string row);

    Eigen::Matrix4f matrixFromRows(std::vector<std::vector<float>> rows);

    pcl::PointXYZ pclFromEigen(Eigen::Vector3f vec);

    std::string type2str(int type);

}

#endif //OBJECTRECONSTRUCTION_HELPERS_H
