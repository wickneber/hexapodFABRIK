//
// Created by root on 5/29/20.
//

#include <iostream>
#include "DOF3Leg.h"
#include "utils.h"

DOF3Leg::DOF3Leg(unsigned int legNumber, double offset, const PositionMatrix& initialPosition):
        idNumber(legNumber), rotationOffset(utils::degreeToRadian(offset)), initPosition(initialPosition), currentPosition(initialPosition),
        goalPosition(initialPosition[initialPosition.size()-1]){}


DOF3Leg::DOF3Leg(const DOF3Leg& rhs){
    idNumber = rhs.idNumber;
    rotationOffset = rhs.rotationOffset;
    initPosition = rhs.initPosition;
    goalPosition = rhs.goalPosition;
}

// Getters
unsigned int DOF3Leg::getLegId() const{
    return idNumber;
}

const PositionMatrix& DOF3Leg::getInitialPosition() const{
    return initPosition;
}

PositionMatrix DOF3Leg::getCurrentPosition() const{
    return currentPosition;
}

Vector13d DOF3Leg::getEndEffector() const{
    return currentPosition[2];
}

std::vector<Eigen::Matrix<double, 1, 2>> DOF3Leg::getYZLinkPositions() const {
    std::vector<Eigen::Matrix<double, 1, 2>> linkPositions;
    for(int row = 0; row < 3; ++row) {
        Eigen::Matrix<double, 1, 2> temp;
        temp << currentPosition[row][1], currentPosition[row][2];
        linkPositions.push_back(temp);
    }
    return linkPositions;
}

Eigen::Matrix<double, 1, 2> DOF3Leg::getYZGoalPosition() const{
    Vector13d goal = getGoalPosition();
    Eigen::Matrix<double, 1, 2> toReturn = {goal[1], goal[2]};
    return toReturn;
}

const Vector13d& DOF3Leg::getGoalPosition() const{
    return goalPosition;
}

double DOF3Leg::getRotationOffset() const{
    return rotationOffset;
}

unsigned int DOF3Leg::getDegreeOfFreedom() const {
    return dof;
}

void DOF3Leg::setInitialPosition(const PositionMatrix& newInitialPosition){
    initPosition = newInitialPosition;
}

void DOF3Leg::setGoalPosition(const Vector13d& newGoalPosition){
    goalPosition = newGoalPosition;
}

void DOF3Leg::setCurrentPosition(const PositionMatrix& newCurrentPosition){
    for(int i = 0; i < dof; ++i){
        currentPosition[i] = newCurrentPosition[i];
    }
}

void DOF3Leg::setCurrentYZPosition(const std::vector<Vector12d>& newCurrentPosition){
    for(int row=0; row < currentPosition.size(); ++row){
        currentPosition[row][1] = newCurrentPosition[row][0];
        currentPosition[row][2] = newCurrentPosition[row][1];
    }
}

void DOF3Leg::setRotationOffset(double newOffset) {
    rotationOffset = utils::degreeToRadian(newOffset);
}