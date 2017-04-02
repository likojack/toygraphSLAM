//
// Created by kejie on 28/03/17.
//

#ifndef TOYGRAPHSLAM_SIMULATOR_H
#define TOYGRAPHSLAM_SIMULATOR_H

#include "Robot.h"

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

#include <math.h>
using namespace std;

class Simulator {
//simulator should generate a map with the size of map and the number of landmark given
	int num_landmark;
	int width;
	int length;
    vector<Point2> landmark;
  public:
	Simulator (int num_mark,int wi,int len) : num_landmark(num_mark), width(wi), length(len) {}
    vector<Point2> generate_landmark(); //return a set of landmark position, an array of Point2
	void begin_simulate(Robot robot, vector<Pose2> odometry, NonlinearFactorGraph graph); // given robot initial position, a series of commands, and the landmark positions
    vector<int> detect_landmark(Robot robot);
    static double addNoise(double variance);

};


#endif //TOYGRAPHSLAM_SIMULATOR_H
