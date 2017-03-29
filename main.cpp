#include <iostream>
#include "Simulator.h"
#include "Robot.h"


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
    Robot robot(detect_range,detect_angle);

    Simulator simulator(num_landmark, length, width);
    int** landmarks = simulator.generate_landmark();
    for(int i=0;i<num_landmark;i++){
        cout << landmarks[i][0] << " " << landmarks[i][1] << endl;
    }
    return 0;
}
