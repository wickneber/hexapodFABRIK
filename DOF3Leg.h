//
// Created by root on 5/29/20.
//

#ifndef KINEMATICSENGINE_DOF3LEG_H
#define KINEMATICSENGINE_DOF3LEG_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "usefulTypedefs.h"

class DOF3Leg {
public:
    DOF3Leg(unsigned int legNumber, double offset, const PositionMatrix& initialPositions);

    DOF3Leg(const DOF3Leg& rhs);
    // Getters
    /// Gets ID number of leg
    /// \return The ID number of the leg
    unsigned int getLegId() const;

    /// Gets the initial positions the robot leg starts at boot
    /// \return A const PositionMatrix with the initial positions.
    const PositionMatrix& getInitialPosition() const;

    /// Gets the current position of the entire leg
    /// \return A PositionMatrix corresponding to the values of the leg links in cartesian space
    PositionMatrix getCurrentPosition() const;

    /// Gets the current position of the end effector in cartesian space
    /// \return A vector containing the position of the end effector
    Vector13d getEndEffector() const;

    /// Gets the Y and Z values for the legs positions. This is mainly used for the fabrik algorithm.
    /// \return  An eigen matrix corresponding to the femur and tibia values of the current leg
    std::vector<Eigen::Matrix<double, 1, 2>> getYZLinkPositions() const;

    /// Gets the Y and Z values for the goal position used mainly for the fabrik algorithm.
    /// \return A new vector of eigen vectors containing the y and z values for the goal position.
    Eigen::Matrix<double, 1, 2> getYZGoalPosition() const;

    /// Gets the current goal position or the value the robot leg needs to go
    /// \return The
    const Vector13d& getGoalPosition() const;

    /// Gets the rotation offset in radians.
    /// \return The rotation offset in radians
    double getRotationOffset() const;

    /// Gets the degree of freedom of the robot leg
    /// \return The legs degree of freedom
    unsigned int getDegreeOfFreedom() const;

    // Setters
    /// Given a new set of initial positions, set them to that value
    /// \param newInitialPosition The new initialPosition PositionMatrix.
    void setInitialPosition(const PositionMatrix& newInitialPosition);

    /// Given a Vector13d set the current goal position to the new one
    /// \param newGoalPosition
    void setGoalPosition(const Vector13d& newGoalPosition);

    /// Sets the current position of the leg to a new one.
    /// \param newCurrentPosition
    void setCurrentPosition(const PositionMatrix& newCurrentPosition);

    /// Sets the current yz position to the existing current position.
    /// \param newCurrentPosition The new y, z value to set to the current position.
    void setCurrentYZPosition(const std::vector<Vector12d>& newCurrentPosition);

    /// Sets the rotation offset to a new rotation offset
    /// \param newOffset A new rotation offset in degrees
    void setRotationOffset(double newOffset);

private:
    unsigned int idNumber = 0;
    double rotationOffset = 0.0;
    const unsigned int dof = 3;
    PositionMatrix initPosition;
    PositionMatrix currentPosition;
    Vector13d goalPosition;
};


#endif //KINEMATICSENGINE_DOF3LEG_H
