//
// Created by root on 5/29/20.
//

#ifndef KINEMATICSENGINE_KINEMATICSENGINE_H
#define KINEMATICSENGINE_KINEMATICSENGINE_H



#include <vector>
#include "DOF3Leg.h"
#include "usefulTypedefs.h"

namespace KinematicsEngine {

    /// Implements a hybrid algorithm that consists of FABRIK inverse kinematics and the
    /// analytic geometric approach to calculate the inverse kinematics values for the
    /// given robot legs.
    /// \param x The desired x value for the robot leg(s) to be.
    /// \param y The desired y value for the robot leg(s) to be.
    /// \param z The desired z value for the robot leg(s) to be.
    /// \param robotLegs The set of legs to calculate the leg kinematics for.
    void doKinematics(double x, double y, double z, std::vector<DOF3Leg*>& robotLegs);

    /// Given a desired translation on the base reference frame, calculate the
    /// resulting x, y, and z translations for the robot legs
    /// \param xTranslation The desired x translation in centimeters
    /// \param yTranslation The desired y translation in centimeters
    /// \param zTranslation The desired z translation in centimeters
    void doTranslation(double xTranslation, double yTranslation, double zTranslation,
                       std::vector<DOF3Leg*>& robotLegs);

    /// Given the yaw, pitch, and roll in radians, calculate the resulting legs
    /// goal positions after all the rotations.
    /// \param yaw The yaw degree to rotate the robot
    /// \param pitch The pitch degree to rotate the robot
    /// \param roll The roll degree to rotate the robot
    void doRotation(double yaw, double pitch, double roll, std::vector<DOF3Leg*>& robotLegs);


    void testFabrik(DOF3Leg* robotLeg);

    /// Given a specific leg id number, use the FABRIK algorithm to calculate the IK of the specific robot leg
    /// \param legId The leg ID number that a calculation is desired
    static void doFabrik(DOF3Leg* robotLeg);

    /// Just a testing function to test the FABRIK algorithm
    /// \param legId the leg to test
    //void testFabrik(std::shared_ptr<DOF3Leg>& robotLeg);

};


#endif //KINEMATICSENGINE_KINEMATICSENGINE_H
