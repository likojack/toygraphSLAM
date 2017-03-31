//
// Created by kejie on 28/03/17.
//

#include "Simulator.h"
using namespace std;

vector<Point2> Simulator::generate_landmark() {
    // declare a 2D array to store the positions of landmark
    vector<Point2> landmark_position;
    // for testing, generate landmark on diagonal line
    for (int i = 0; i < num_landmark; i++) {
        landmark_position.push_back(Point2(i,i));
    }
    landmark = landmark_position;
    //generate random position for landmarks
//    srand(time(NULL));
//    for(int i=0;i<num_landmark;i++){
//        landmark_position[i][0] = rand()%length;
//        landmark_position[i][1] = rand()%width;
//    }
    return landmark_position;
//}
}

vector<Point2> Simulator::detect_landmark(Robot robot) {
    // detect if landmakr in detect range, assume detect angle is 360
    Pose2 robot_position = robot.get_position();
    vector<float> detected;
    for(int i = 0; i < num_landmark; i++) {
        double x_landmark = landmark[i].x();
        double y_landmark = landmark[i].y();
        if(sqrt((x_landmark-robot_position.x())**2 + (y_landmark - robot_position.y())**2) < robot.detect_range) {
            detected.push_back(landmark[i]);
        }
    }
    // return landmark index
    return detected
}

// odometry is a set of command for moving
void Simulator::begin_simulate(Robot robot, vector<Pose2> odometry, vector<Point2> landmarks, NonlinearFactorGraph graph) {
    // create noise model: 
    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3)<< 0.2, 0.2, 0.1));
    // add symbol for every landmark:
//     for i in enumerate(landmark): li = symbol('l',i);

    // add prior node with noise, noise = normal_distribution(robot.variance):
    // graph.add(PriorFactor<Pose2>(c1, Pose2(0,0,0), priorNoise));
    Symbol c1('c',1);
    Pose2 prior(0.0, 0.0, 0.0); // prior mean is at origin
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    graph.add(PriorFactor<Pose2>(c1, prior, priorNoise)); // add directly to graph
    // run detect_landmark, add camera-landmark constraint:

    // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));


    // for i in odometry:
    int num_odometry = odometry.size();
    for(int i = 0 ; i < num_odometry;i++){

    }
        // run position = robot.move(i); (with noise)
        // add factor between two poses:
        // ci = symbol('c',i);
        // graph.add(BetweenFactorPose2(c_i-1, ci, odometry, odometryNoise));
        // run detect_landmark, add camera-landmark constraint:
        // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));
}


