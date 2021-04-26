#include <iostream>
#include <eigen3/Eigen/Dense>
#include "DOF3Leg.h"
#include "usefulTypedefs.h"
#include "KinematicsEngine.h"

void printPosition(DOF3Leg* leg){
    std::cout << "\n=============================================\n";
    std::cout << "|              Position Check               |\n";
    std::cout << "=============================================\n";

    for(auto position: leg->getCurrentPosition()){
        std::cout << "Position " << position << std::endl;
    }
    std::cout << "=============================================\n\n";
}

int main() {
    PositionMatrix init_positions = {
            Vector13d(0,0,0),
            Vector13d(0, 3,6.9),
            Vector13d(0, 10, 0)};

    auto* leg1 = new DOF3Leg(1, 45, init_positions);
    auto* leg4 = new DOF3Leg(4, -135, init_positions);
    auto* leg5 = new DOF3Leg(5, 90, init_positions);

    Vector13d goal = {0,3,3};
    leg1->setGoalPosition(goal);
    Vector13d goal1 = {0, 4, 2};
    leg4->setGoalPosition(goal1);
    leg5->setGoalPosition(goal);

    std::vector<DOF3Leg*> robotLegs = {leg1, leg4, leg5};
    KinematicsEngine::doKinematics(3, 3, 3, robotLegs);
    /*KinematicsEngine::testFabrik(leg1);
    KinematicsEngine::testFabrik(leg4);
    KinematicsEngine::testFabrik(leg5);
    */
    for(int i = 0; i < 3; ++i){
        printPosition(robotLegs[i]);
    }

    Vector13d goal2 = {0, 5, 1};
    leg1->setGoalPosition(goal2);

    delete leg1;
    delete leg4;
    delete leg5;

    return 0;
}
