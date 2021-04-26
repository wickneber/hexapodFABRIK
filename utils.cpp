//
// Created by root on 5/29/20.
//

#include "utils.h"

double utils::degreeToRadian(double degree){
    return (degree * M_PI) / 180;
}

double utils::radianToDegree(double radian){
    return (radian * 180)/M_PI;
}

double utils::constrain(double lowerBound, double upperBound, double toConstrain){
    if(lowerBound <= toConstrain && toConstrain <= upperBound)
        return toConstrain;

    if(toConstrain < lowerBound)
        return lowerBound;

    if(upperBound < toConstrain)
        return upperBound;

}

double utils::getEuclideanDistance(const Eigen::MatrixXd &mat1, const Eigen::MatrixXd &mat2) {
        return (mat1 - mat2).norm();
}

std::vector<double> utils::getLegLengths(const PositionMatrix &legPositions) {
    std::vector<double> legLengths;
    for(int i = 0; i < legPositions.size()-1; ++i)
        legLengths.push_back(getEuclideanDistance(legPositions[i], legPositions[i + 1]));

    return legLengths;
}

void utils::joinThreads(std::vector<std::thread>& threadsToJoin) {
    for(auto& toJoin: threadsToJoin) {
        if(toJoin.joinable())
            toJoin.join();
    }
}