//
// Created by kejie on 29/03/17.
//

#include <random>
#include "Robot.h"

using namespace std;
using namespace gtsam;

float* Robot::report_position() {
    float* position = new float[3];
    position[0] = x;
    position[1] = y;
    position[2] = theta;
    return position;
}

float Robot::move(int* command) {
    default_random_engine generator;
    normal_distribution distribution_x(0,variance);
    normal_distribution distribution_y(0,variance);
    normal_distribution distribution_theta(0,variance);
    Point2 pt;
}
