//
// Created by kejie on 29/03/17.
//

#include "Robot.h"

using namespace std;

float* Robot::report_position() {
    float* position = new float[3];
    position[0] = x;
    position[1] = y;
    position[2] = theta;
    return position;
}

Pose2 Robot::move(Pose2 odometry) {
    Point2 pt;
}
