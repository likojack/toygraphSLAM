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

    float detect_range = 2.0;
    float detect_angle = pi/2;
    float initial_x = 0, initial_y = 0 , initial_theta = 0;
    NonlinearFactorGraph graph;
    Symbol x1('x',1);

    Pose2 prior(0.0, 0.0, 0.0); // prior mean is at origin
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    graph.add(PriorFactor<Pose2>(x1, prior, priorNoise)); // add directly to graph
    graph.print("Factor Graph:\n");

    Pose2 test_odometry(2,0,0);
    Simulator simulator(num_landmark,width,length);
    Robot robot(detect_range, detect_angle,initial_x, initial_y, initial_theta);
    vector<Point2> landmarks = simulator.generate_landmark();
    for(int i = 0 ; i < num_landmark;i++){
        cout << landmarks[i].x() << " " << landmarks[i].y() << endl;
    }

}
