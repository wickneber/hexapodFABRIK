//
// Created by root on 5/29/20.
//


#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include "DOF3Leg.h"

class DOF3Leg;

typedef Eigen::Matrix<double, 1, 3> Vector13d;
typedef std::vector<Vector13d> PositionMatrix;
typedef std::vector<DOF3Leg> LegVector;
typedef std::vector<int> IdVector;
typedef Eigen::Matrix<double, 1, 2> Vector12d;
typedef std::map<int, DOF3Leg> LegMap;

