//
// Created by kejie on 28/03/17.
//

#include "Simulator.h"
using namespace std;
// use Pose2 to store landmark, the last element for index
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

vector<int> Simulator::detect_landmark(Robot robot) {
    // detect if landmark in detect range, assume detect angle is 360
    Pose2 robot_position = robot.getPosition();
    //detected store the distance and angle between detected landmark and robot, which will be used in factor graph
    vector<int> detected;
    for(int i = 0; i < num_landmark; i++) {
        double angle = 0;
        if(robot.getPosition().range(landmark[i]) < robot.getDetect_range()) {
            detected.push_back(i);
        }
    }
    // return landmark index
    return detected;
}

// odometry is a set of command for moving
void Simulator::begin_simulate(Robot robot, vector<Pose2> odometry, NonlinearFactorGraph graph) {
    // create noise model: 
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
    // create factor graph symbol for every camera pose and landmark
    vector<Symbol> odometry_symbol;
    vector<Symbol> landmark_symbol;
    for(int i=0;i<odometry.size();i++){
        odometry_symbol.push_back(Symbol('c',i));
        odometry_symbol[i].print("odometry symbol: ");
    }
    for(int i=0;i<num_landmark;i++){
        landmark_symbol.push_back(Symbol('l',i));
        landmark_symbol[i].print("landmark symbol: ");
    }
    graph.add(PriorFactor<Pose2>(odometry_symbol[0], robot.getPosition(), priorNoise));
    vector<int> detected = detect_landmark(robot);
    //testing: what are the landmarks detected
    for(int i = 0; i < detected.size();i++){
        cout << detected[i] << endl;
    }
    if(!detected.empty()){
        for(int i = 0 ; i < detected.size();i++){
            graph.add(BearingRangeFactor<Pose2, Point2>(odometry_symbol[0], landmark_symbol[detected[i]], robot.getPosition().bearing(landmark[detected[i]]), robot.getPosition().range(landmark[detected[i]]), measurementNoise));
        }
    }
    graph.print("graph at the initial position: \n");
//    //robot start moving
//    int num_odometry = odometry.size();
//    for(int i = 1 ; i < num_odometry;i++){
//        Pose2 delta_position = odometry[i-1];
//        Pose2 old_position = robot.getPosition();
//        double x_new = old_position.compose(delta_position).x();
//        double y_new = old_position.compose(delta_position).y();
//        double theta_new = old_position.compose(delta_position).theta();
//        robot.setPosition(x_new, y_new,theta_new);
//
//        graph.add(BetweenFactor<Pose2>(odometry_symbol[i-1], odometry_symbol[i], odometry[i-1], odometryNoise));
//        vector<int> detected = detect_landmark(robot);
//        if(!detected.empty()){
//            for(int j = 0 ; j < detected.size();j++){
//                graph.add(BearingRangeFactor<Pose2, Point2>(odometry_symbol[i], landmark_symbol[detected[j]], robot.getPosition().bearing(landmark[detected[j]]), robot.getPosition().range(landmark[detected[j]]), measurementNoise));
//            }
//        }
//    }
        // run position = robot.move(i); (with noise)
        // add factor between two poses:
        // ci = symbol('c',i);
        // graph.add(BetweenFactorPose2(c_i-1, ci, odometry, odometryNoise));
        // run detect_landmark, add camera-landmark constraint:
        // graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, noiseModel));
}


