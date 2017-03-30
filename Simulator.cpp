//
// Created by kejie on 28/03/17.
//

#include "Simulator.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
int** Simulator::generate_landmark() {
    // declare a 2D array to store the positions of landmark
    int** landmark_position = new int* [num_landmark];
    for(int i=0; i<num_landmark;i++){
        landmark_position[i] = new int[2];
    }
    // for testing, generate landmark on diagonal line
    for(int i=0;i<num_landmark;i++){
        landmark_position[i][0] = i;
        landmark_position[i][1] = i;
    }
    //generate random position for landmarks
//    srand(time(NULL));
//    for(int i=0;i<num_landmark;i++){
//        landmark_position[i][0] = rand()%length;
//        landmark_position[i][1] = rand()%width;
//    }
    return landmark_position;
}

int* Simulator::detect_landmark(Robot robot) {
    // detect if landmakr in detect range
    // return landmark index
}

// odometry is a set of command for moving
void Simulator::begin_simulate(Robot robot, Pose2* odometry, Point2* landmarks, Graph NonlinearGraph) {
    // create noise model: 
    // noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
    // add symbol for every landmark:
    // for i in enumerate(landmark): li = symbol('l',i);

    // add prior node with noise, noise = normal_distribution(robot.variance):
    // graph.add(PriorFactor<Pose2>(c1, Pose2(0,0,0), priorNoise));

    // run detect_landmark, add camera-landmark constraint:
    // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));

    // for i in odometry:
        // run position = robot.move(i); (with noise)
        // add factor between two poses:
        // ci = symbol('c',i);
        // graph.add(BetweenFactorPose2(c_i-1, ci, odometry, odometryNoise));
        // run detect_landmark, add camera-landmark constraint:
        // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));
}


