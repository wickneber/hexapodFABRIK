//
// Created by root on 5/29/20.
//

#ifndef KINEMATICSENGINE_UTILS_H
#define KINEMATICSENGINE_UTILS_H

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <thread>
#include "usefulTypedefs.h"

namespace utils{

    double degreeToRadian(double degree);
    double radianToDegree(double radian);

    double constrain(double lowerBound, double upperBound, double toConstrain);

    double getEuclideanDistance(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);

    std::vector<double> getLegLengths(const PositionMatrix& legPositions);

    void joinThreads(std::vector<std::thread>& threadsToJoin);

}


#endif //KINEMATICSENGINE_UTILS_H
