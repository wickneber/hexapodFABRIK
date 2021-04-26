//
// Created by root on 5/29/20.
//

#include "KinematicsEngine.h"
#include "utils.h"

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>

#ifndef THRESHOLD
// Crazy threshold test to see how long it would take to solve the solution ;)
#define THRESHOLD 0.000000000000000000000000000000000000000000000000000000000000000000001
#endif

// anonymous namespace to contain all the pitch yaw roll calculations
namespace{ // START OF ANON NAMESPACE
    typedef Eigen::Matrix<double, 1, 3> FilterVec;
    typedef Eigen::Array<double, 1, 3> ArrayVec;

    void calculateLegTranslation(const Vector13d& translation, DOF3Leg* robotLeg, const FilterVec& filter);
    void calculateZTranslation(double z, DOF3Leg* robotLeg);
    void calculateYaw(double yaw, DOF3Leg* robotLeg);
    void calculatePitch(double pitch, DOF3Leg* robotLeg);
    void calculateRoll(double roll, DOF3Leg* robotLeg);
    Eigen::Matrix3d getZRotation(double angleToRotate);
    Eigen::Matrix3d getYRotation(double angleToRotate);
    Eigen::Matrix3d getXRotation(double angleToRotate);
    void legTranslationThread(double xTranslation, double yTranslation, double zTranslation, DOF3Leg* robotLeg);
    void doXYCalculations(DOF3Leg* robotLeg);
    void kinematicsThread(double xTranslation, double yTranslation, double zTranslation, DOF3Leg* robotLeg);


    void calculateLegTranslation(const Vector13d& translation, DOF3Leg* robotLeg, const FilterVec& filter){
        auto legGoalPosition = robotLeg->getGoalPosition().array();
        unsigned int legID = robotLeg->getLegId();

        // if the leg is not one of the middle legs;
        if(legID != 5 && legID != 6){
            ArrayVec filteredTranslation = translation.array() * filter.array();
            robotLeg->setGoalPosition((legGoalPosition + filteredTranslation).matrix());
            return;
        }
        // if the robot is a middle leg
        robotLeg->setGoalPosition(legGoalPosition.matrix() + translation);
    }

    void calculateZTranslation(double z, DOF3Leg* robotLeg){
        // First constrian the z value between acceptable bounds
        z = utils::constrain(-5, 5, z);
        // Get the goal position
        Vector13d currentGoalPosition = robotLeg->getGoalPosition();
        // modify the z value
        currentGoalPosition[2] += z;
        // set it to the newly modified z value
        robotLeg->setGoalPosition(currentGoalPosition);

    }

    void calculateYaw(double yaw, DOF3Leg* robotLeg){

        std::cout << "\n=============================================" << std::endl;
        std::cout << "============ CalculateYaw   called ==========" << std::endl;
        std::cout << "=============================================\n" << std::endl;
        std::cout << "LEG: " << robotLeg->getLegId() << std::endl;
        Eigen::Matrix3d zRotation = getZRotation(yaw);
        Vector13d currentGoal = robotLeg->getGoalPosition();
        std::cout << "Current goal: " << currentGoal << std::endl;
        Vector13d rotationGoal = zRotation * currentGoal.transpose();
        std::cout << "Rotation goal: " << rotationGoal << std::endl;
        robotLeg->setGoalPosition(rotationGoal);
    }

    void calculatePitch(double pitch, DOF3Leg* robotLeg){

        std::cout << "\n=============================================" << std::endl;
        std::cout << "============ CalculatePitch called ==========" << std::endl;
        std::cout << "=============================================\n" << std::endl;
        std::cout << "LEG: " << robotLeg->getLegId() << std::endl;

        std::cout << "Leg goal pos: " << robotLeg->getGoalPosition() << std::endl;
        Eigen::Matrix3d xRotation = getXRotation(pitch);
        std::cout << "Pitch Rotation:\n" << xRotation << std::endl;
        Vector13d currentGoalPosition = robotLeg->getGoalPosition();

        Vector13d pitchedGoal = xRotation * currentGoalPosition.transpose();
        std::cout << "\nPitched Goal:\n" << pitchedGoal << std::endl;

        // Get the difference between the pitched goal and the current goal position
        Vector13d difference = pitchedGoal - currentGoalPosition;

        // We only want the difference on the z axis so we throw out the rest.
        difference[0] = 0;
        difference[1] = 0;

        unsigned int legId = robotLeg->getLegId();
        if(legId == 1 || legId == 2){
            // just want the z value information for the front two leg pitch
            currentGoalPosition[2] = pitchedGoal[2];
            //std::cout << "Current goal position: " << currentGoalPosition << std::endl;
            robotLeg->setGoalPosition(currentGoalPosition);
            //std::cout << "Front legs final: " << robotLeg.getGoalPosition() << std::endl;
            return;
        }

        if(legId == 3 || legId == 4){
            robotLeg->setGoalPosition(currentGoalPosition - difference);
            //std::cout << "Back leg final: " << robotLeg.getGoalPosition() << std::endl;
        }

    };

    void calculateRoll(double roll, DOF3Leg* robotLeg){

        std::cout << "\n=============================================" << std::endl;
        std::cout << "============ CalculateRoll called ==========" << std::endl;
        std::cout << "=============================================\n" << std::endl;

        // first get the matrix corresponding to the rotation about the x axis
        Eigen::Matrix3d xRotation = getXRotation(roll);
        Vector13d currentGoal = robotLeg->getGoalPosition();
        Vector13d rolledGoal  = xRotation * currentGoal.transpose();

        Vector13d difference = rolledGoal - currentGoal;
        std::cout << "Current Goal: " << currentGoal << std::endl;
        std::cout << "Rolled Goal: " << rolledGoal << std::endl;
        std::cout << "Difference: " << difference << std::endl;

        unsigned int legId = robotLeg->getLegId();

        // if the robot leg is on the left side of the body
        if(legId == 1 || legId == 3 || legId == 5){
            currentGoal[2] = difference[2];
            robotLeg->setGoalPosition(currentGoal);
            std::cout << "FINAL ROLL VALUE: " << robotLeg->getGoalPosition() << std::endl;
            return;
        }
        // else the robot leg is on the right side of the body

        // only want the z position so we zero out the x and y position of difference
        difference[0] = 0;
        difference[1] = 0;

        std::cout << "Difference after: " << difference << std::endl;
        robotLeg->setGoalPosition(currentGoal - difference);
        std::cout << "FINAL ROLL VALUE: " << robotLeg->getGoalPosition() << std::endl;
    }

    /*
     * 3d rotation matrix about the z axis
     */
    Eigen::Matrix3d getZRotation(double angleToRotate){
        Eigen::Matrix3d zRotation;
        zRotation << std::cos(angleToRotate), -1*std::sin(angleToRotate), 0,
                std::sin(angleToRotate), std::cos(angleToRotate), 0,
                0, 0, 1;
        return zRotation;
    }
    /*
     * 3d rotation matrix about the y axis
     */
    Eigen::Matrix3d getYRotation(double angleToRotate){
        Eigen::Matrix3d yRotation;
        yRotation << std::cos(angleToRotate), 0, std::sin(angleToRotate),
                0, 1, 0,
                -1*std::sin(angleToRotate), 0, std::cos(angleToRotate);
        return yRotation;
    }

    /*
     * 3d rotation matrix about the x axis
     */
    Eigen::Matrix3d getXRotation(double angleToRotate){
        Eigen::Matrix3d xRotation;
        xRotation << 0,0,1,
                0, std::cos(angleToRotate), -1*std::sin(angleToRotate),
                0, std::sin(angleToRotate), std::cos(angleToRotate);
        return xRotation;
    }

    void legTranslationThread(double xTranslation, double yTranslation, double zTranslation, DOF3Leg* leg){

        Eigen::Matrix3d zRotation = getZRotation(leg->getRotationOffset());
        //std::cout << "zRotation: \n" << zRotation << std::endl;

        // First obtian the y translation component and rotate it tot the legs local reference frame
        Vector13d yTranslationComponent = {0, yTranslation, 0};
        Vector13d yLocalFrameTranslation = zRotation * yTranslationComponent.transpose();

        // Second, obtain the x translation component and rotate it into the legs reference frame
        Vector13d xTranslationComponent = {xTranslation, 0, 0};
        Vector13d xLocalFrameTranslation = zRotation * xTranslationComponent.transpose();

        /* =========================================================
         * ============== Compute y component translation ==========
         * =========================================================
         */
        Vector13d yFilter = {1, -1, 0};
        calculateLegTranslation(yLocalFrameTranslation, leg, yFilter);

        /* ==========================================================
         * ============ Compute x component translation =============
         * =========================================================
         */
        Vector13d xFilter = {-1, 1, 0};
        calculateLegTranslation(xLocalFrameTranslation, leg, xFilter);

        /* =============================================================
         * ============ Compute the z component translation ============
         * =============================================================
         */
        //std::cout << "LEG GOAL: " << leg.getLegId() << " " << leg.getGoalPosition() << std::endl;
        calculateZTranslation(zTranslation, leg);
    }

    void doXYCalculations(DOF3Leg* robotLeg){
        // init stuff just gets the leg goal and current value of the given robot leg
        auto legGoal = robotLeg->getGoalPosition();
        auto currentValue = robotLeg->getEndEffector();

        // from the legGoal and currentValue, create two 2d vectors corresponding to just the x and y coordinates of
        // the robot leg positions
        Eigen::Matrix<double, 1, 2> legGoalxy, currentValuexy;
        legGoalxy << legGoal[0], legGoal[1];
        currentValuexy << currentValue[0], currentValue[1];

        // determine the angle of rotation by getting the cross product of the two original vectors
        // if the z axis is positive, rotate in a clockwise direction if not rotate in a counter clockwise direction
        int rotationDirection = 1;
        if (currentValue.cross(legGoal)[2] >= 0)
            rotationDirection = -1;

        // calculate the angle of rotation
        auto dotted = legGoalxy.dot(currentValuexy);
        auto mag = legGoalxy.norm() * currentValuexy.norm();
        auto tot = rotationDirection*acos(dotted/mag);

        // the actual zRotation matrix to multiply the current leg positions with
        Eigen::Matrix2d zRotation2d;
        zRotation2d << cos(tot), sin(tot), -sin(tot), cos(tot);

        std::vector<Vector13d> currentValues = robotLeg->getCurrentPosition();

        // calculate the new value after rotation
        for(int i = 1; i < robotLeg->getDegreeOfFreedom(); ++i){
            Eigen::Matrix<double, 2, 1> linkXYPostion;
            linkXYPostion << currentValues[i][0], currentValues[i][1];
            auto rotatedValue = zRotation2d * linkXYPostion;
            currentValues[i][0] = rotatedValue[0];
            currentValues[i][1] = rotatedValue[1];
        }
        // after the new rotation value is calculated, set the new value into the robot leg
        robotLeg->setCurrentPosition(currentValues);
    }

    void kinematicsThread(DOF3Leg* robotLeg){
        KinematicsEngine::doFabrik(robotLeg);
        doXYCalculations(robotLeg);
        std::cout << "Done with kinematics thread\n";
    }

} // END OF ANON NAMESPACE


void KinematicsEngine::doKinematics(double x, double y, double z, std::vector<DOF3Leg*>& robotLegs){
    /*std::vector<std::thread> threadVector;
    for (DOF3Leg* leg: robotLegs){
        threadVector.emplace_back(std::thread(kinematicsThread, leg));
    }*/
    Vector13d goal = {x, y, z};
    for(DOF3Leg* leg: robotLegs){
        leg->setGoalPosition(goal);
        //doXYCalculations(leg);
        doFabrik(leg);
        doXYCalculations(leg);
    }
}

void KinematicsEngine::doTranslation(double xTranslation, double yTranslation, double zTranslation,
                                     std::vector<DOF3Leg*>& robotLegs) {
    std::vector<std::thread> legThreads;
    /*
    for(auto & robotLeg: robotLegs){
        legThreads.emplace_back(std::move(std::thread(legTranslationThread, xTranslation, yTranslation, zTranslation,
                                                      std::ref(robotLeg))));
    }
    utils::joinThreads(legThreads);*/
}

void KinematicsEngine::doRotation(double yaw, double pitch, double roll,
                                  std::vector<DOF3Leg*>& robotLegs) {
    double yawRadians   = utils::degreeToRadian(yaw);
    double pitchRadians = utils::degreeToRadian(pitch);
    double rollRadians  = utils::degreeToRadian(roll);

    for(DOF3Leg* leg: robotLegs){
        calculateYaw(yawRadians, leg);
        calculatePitch(pitchRadians, leg);
        calculateRoll(rollRadians, leg);
    }


}

void KinematicsEngine::doFabrik(DOF3Leg* robotLeg){
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
    std::cout << "Leg#" << robotLeg->getLegId() << ": is currently being processed.\n";
    //std::cout << "Top of FABRIK" << std::endl;
    std::vector<Vector12d> currentLegPositions = robotLeg->getYZLinkPositions();
    //std::cout << "Link positions\n";
    Vector12d base = currentLegPositions[0];
    //std::cout << "BASE: " << base << std::endl;
    std::vector<double> legLengths = utils::getLegLengths(robotLeg->getCurrentPosition());

    double totalLength = std::accumulate(legLengths.begin(), legLengths.end(), 0.0);

    Vector12d goalPosition = robotLeg->getYZGoalPosition();
    double desiredDistance = utils::getEuclideanDistance(base, goalPosition);
    //std::cout << "Right before checking to see if the desired point is within the range\n";
    std::cout << "Total length: " << totalLength << " desiredDistance " << desiredDistance << std::endl;
    if(totalLength < desiredDistance){
        std::cout << "Distance too large! " << "Desired distance: " << desiredDistance << ". Leg total reach: " << totalLength << std::endl;
        for(int i = 0; i < currentLegPositions.size()-1; ++i){
            double r = utils::getEuclideanDistance(goalPosition, currentLegPositions[i]);
            double mu = legLengths[i] / r;
            currentLegPositions[i+1] = (1-mu)*currentLegPositions[i] + mu*goalPosition;
        }

        robotLeg->setCurrentYZPosition(currentLegPositions);
        return;
    }

    // just to time how long it takes to solve
    auto start = std::chrono::high_resolution_clock::now();

    int numLinks = currentLegPositions.size();
    double difference = utils::getEuclideanDistance(currentLegPositions[numLinks-1], goalPosition);
    unsigned int iteration = 0;
    while(difference > THRESHOLD){
        // Forward reach
        currentLegPositions[numLinks-1] = goalPosition;
        for(int i = numLinks-2; i >= 0; --i){
            double r = utils::getEuclideanDistance(currentLegPositions[i], currentLegPositions[i+1]);
            double mu = legLengths[i] / r;
            currentLegPositions[i] = (1 - mu)*currentLegPositions[i+1] + mu*currentLegPositions[i];
        }

        // Backward reach
        currentLegPositions[0] = base;
        for(int i = 0; i < currentLegPositions.size()-1; ++i){
            double r = utils::getEuclideanDistance(currentLegPositions[i], currentLegPositions[i+1]);
            double mu = legLengths[i] / r;
            currentLegPositions[i+1] = (1-mu) * currentLegPositions[i] + mu * currentLegPositions[i+1];
        }

        difference = utils::getEuclideanDistance(currentLegPositions[numLinks-1], goalPosition);
        robotLeg->setCurrentYZPosition(currentLegPositions);
        iteration += 1;
    }

    for(const auto& position: robotLeg->getCurrentPosition()){
        std::cout << "Position: " << position << " " << std::endl;
    }
    std::cout << "Iterations: " << iteration << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time: " << diff.count() << std::endl;
    std::cout << "=============================================\n";
}

void KinematicsEngine::testFabrik(DOF3Leg* robotLeg){
    //std::cout << "Testing leg # " << robotLeg->getLegId() << std::endl;
    doFabrik(robotLeg);
}