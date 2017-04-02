#include <iostream>
#include "Simulator.h"


using namespace std;

int main(int argc, char *argv[]) {
    // input an initial position and a series of moving commands
    // initial position in the format of Pose2(x,y,theta)
    // command in the format of (r,d), d = moving length and r = rotation, both with gaussian noise
    // contain a simulator class 
    // contain a robot class with the attribute of detect range
    // import gtsam as SLAM backend

    //user input robot initial position, # landmark, and map size, a series of command
    const double pi = 3.1415926;
    int num_landmark = 4;
    int width = 5;
    int length = 5;
    double detect_range = 2.0;
    double detect_angle = pi/2;
    Pose2 initial_position(0,0,0);
    NonlinearFactorGraph graph;

    vector<Pose2> test_odometry;
    test_odometry.push_back(Pose2(2,0,0));
    Simulator simulator(num_landmark,width,length);
    vector<Point2> landmarks = simulator.generate_landmark();
    //testing landmark position
    for(int i = 0; i < landmarks.size();i++){
        landmarks[i].print();
    }
    Robot robot(detect_range, detect_angle,initial_position.x(),initial_position.y(),initial_position.theta());
    simulator.begin_simulate(robot,test_odometry,graph);

//    Pose2 robot(0,0,Rot2::fromDegrees(45).theta()); // Pose2.theta() is operated under radians
//    Point2 landmark(2,0);
//    Pose2 odometry(2,0,0);
//    cout << robot.range(landmark)<< endl;
//    robot.compose(odometry).print("robot move");
//    cout << robot.x() << endl;
//    cout << pow(landmark.x(),2);


}
