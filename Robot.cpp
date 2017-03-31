//
// Created by kejie on 29/03/17.
//

#include "Robot.h"

using namespace std;

Pose2 Robot::get_position() {
    return position;
}

Pose2 Robot::move(Pose2 odometry) {
    Point2 pt;
}

void set_position(Pose2 odometry) {
	position = odometry;
}
