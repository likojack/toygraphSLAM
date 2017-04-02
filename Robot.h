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
private:
    double detect_range;
    double detect_angle;
    double x;
    double y;
    double theta;


public:
    double getDetect_range() const;
    void setPosition(double x_new, double y_new, double theta_new);
    double getDetect_angle() const;
    Pose2 getPosition();

    Robot(double detect_range, double detect_angle) : detect_range(detect_range), detect_angle(detect_angle) {}
    Robot(double detect_range, double detect_angle, double x, double y , double theta) : detect_range(detect_range),
                                                                                          detect_angle(detect_angle), x(x), y(y), theta(theta) {}
    Pose2 move(Pose2 odometry); // input a command (r,d), output a position and pose (x,y,theta) with noise

};




#endif //TOYGRAPHSLAM_ROBOT_H
