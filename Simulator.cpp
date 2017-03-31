//
// Created by kejie on 28/03/17.
//

#include "Simulator.h"
using namespace std;
// use Pose2 to store landmark, the last element for index
vector<Pose2> Simulator::generate_landmark() {
    // declare a 2D array to store the positions of landmark
    vector<Pose2> landmark_position;
    // for testing, generate landmark on diagonal line
    for (int i = 0; i < num_landmark; i++) {
        landmark_position.push_back(Pose2(i,i,i));
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

vector<Pose2> Simulator::detect_landmark(Robot robot) {
    // detect if landmakr in detect range, assume detect angle is 360
    Pose2 robot_position = robot.get_position();
    //detected store the distance and angle between detected landmark and robot, which will be used in factor graph
    vector<Pose2> detected;
    for(int i = 0; i < num_landmark; i++) {
        double x_landmark = landmark[i].x();
        double y_landmark = landmark[i].y();
        double index_landmark = landmark[i].theta();
        double distance = sqrt((x_landmark-robot_position.x())**2 + (y_landmark - robot_position.y())**2);
        double angle = 
        if(distance < robot.detect_range) {
            detected.push_back(Pose2(distance, angle, index_landmark));
        }
    }
    // return landmark index
    return detected;
}

// odometry is a set of command for moving
void Simulator::begin_simulate(Robot robot, vector<Pose2> odometry, vector<Point2> landmarks, NonlinearFactorGraph graph) {
    // create noise model: 
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta

    // create factor graph symbol for every camera pose and landmark
    vector<Symbol> odometry_symbol;
    vector<Symbol> landmark_symbol;
    for(int i=0;i<odometry.size();i++){
        odometry_symbol.push_back(Symbol('c',i));
    }
    for(int i=0;i<landmarks.size();i++){
        landmark_symbol.push_back(Symbol('l',i));
    }   

    // add symbol for every landmark:
//     for i in enumerate(landmark): li = symbol('l',i);

    // add prior node with noise, noise = normal_distribution(robot.variance):
    // graph.add(PriorFactor<Pose2>(c1, Pose2(0,0,0), priorNoise));
    Pose2 prior(0.0, 0.0, 0.0); // prior mean is at origin
    graph.add(PriorFactor<Pose2>(odometry_symbol[0], prior, priorNoise)); // add directly to graph
    // run detect_landmark, add camera-landmark constraint:
    vector<float> detected = detect_landmark(robot);
    if(!detected.empty()){
        for(int i = 0 ; i < detected.size();i++){
            graph.add(BearingRangeFactor2D(odometry_symbol[0], landmark_symbol[detected[i].theta()], detected[i].y(), detected[i].x(), measurementNoise));
        }
    }
    //robot start moving
    int num_odometry = odometry.size();
    for(int i = 1 ; i < num_odometry;i++){
        Pose2 delta_position = odometry[i-1];
        Pose2 old_position = robot.get_position();
        Pose2 new_position;
        //some calculation needed to calculate the new position of robot
        robot.set_position(new_position);

        graph.add(BetweenFactor<Pose2>(odometry_symbol[i-1], odometry_symbol[i], odometry[i-1], odometryNoise));
        vector<float> detected = detect_landmark(robot);
        if(!detected.empty()){
            for(int j = 0 ; j < detected.size();j++){
                graph.add(BearingRangeFactor2D(odometry_symbol[i], landmark_symbol[detected[j].theta()], detected[j].y(), detected[j].x(), measurementNoise));
            }     
        }   
    }
        // run position = robot.move(i); (with noise)
        // add factor between two poses:
        // ci = symbol('c',i);
        // graph.add(BetweenFactorPose2(c_i-1, ci, odometry, odometryNoise));
        // run detect_landmark, add camera-landmark constraint:
        // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));
}


