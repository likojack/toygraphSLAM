//
// Created by kejie on 29/03/17.
//

#ifndef TOYGRAPHSLAM_ROBOT_H
#define TOYGRAPHSLAM_ROBOT_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>


using namespace gtsam;


class Robot {
    float detect_range;
    float detect_angle;
    Pose2 position;
public:
    Robot(float detect_range, float detect_angle) : detect_range(detect_range), detect_angle(detect_angle) {}
    Robot(float detect_range, float detect_angle, Pose2 position) : detect_range(detect_range),
                                                                                          detect_angle(detect_angle), position(position) {}
    Pose2 get_position();

    Pose2 move(Pose2 odometry); // input a command (r,d), output a position and pose (x,y,theta) with noise

};




#endif //TOYGRAPHSLAM_ROBOT_H
