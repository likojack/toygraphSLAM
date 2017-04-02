//
// Created by kejie on 29/03/17.
//

#include "Robot.h"

using namespace std;

Pose2 Robot::getPosition() {
    Pose2 position(x,y,theta);
    return position;
}

void Robot::setPosition(double x, double y, double theta) {
    Robot::x = x;
    Robot::y = y;
    Robot::theta = theta;

}

double Robot::getDetect_range() const {
    return detect_range;
}

double Robot::getDetect_angle() const {
    return detect_angle;
}

Pose2 Robot::move(Pose2 odometry) {
    Point2 pt;
}

