//
// Created by kejie on 28/03/17.
//

#include "Simulator.h"
using namespace std;
// use Pose2 to store landmark, the last element for index
double Simulator::addNoise(double variance){
    random_device rd;
    mt19937 e2(rd());
    normal_distribution<double> distribution(0,variance);
    double number = 0;
    do {
        number = distribution(e2);
    }while(number > 0.5);
    return number;
}
vector<Point2> Simulator::generate_landmark() {
    // declare a 2D array to store the positions of landmark
    vector<Point2> landmark_position;
//    landmark_position.push_back(Point2(0.5,0.5));

    //generate random position for landmarks
    srand(time(NULL));
    for(int i=0;i<num_landmark;i++){
        landmark_position.push_back(Point2(rand()%length, rand()%width));
    }
    landmark = landmark_position;
    return landmark_position;
}


vector<int> Simulator::detect_landmark(Robot robot) {
    // detect if landmark in detect range, assume detect angle is 360
    Pose2 robot_position = robot.getPosition();
    //detected store the distance and angle between detected landmark and robot, which will be used in factor graph
    vector<int> detected;
    for(int i = 0; i < num_landmark; i++) {
        double angle = 0;
        //assume known data associate
        //TODO: detect angle can be functioning if Pose2.bearing added
        if((robot.getPosition().range(landmark[i]) <= robot.getDetect_range()) and (robot.getPosition().bearing(landmark[i]).degrees()<=90 and robot.getPosition().bearing(landmark[i]).degrees()>=-90)) {
            detected.push_back(i);
        }
    }
    // return landmark index
    return detected;
}

// odometry is a set of command for moving
void Simulator::begin_simulate(Robot robot, vector<Pose2> odometry, NonlinearFactorGraph graph) {
    // create noise model:
    double prior_x = 0.20, prior_y = 0.2, prior_theta = 0.1;
    double x_variance = 0.2, y_variance = 0.2, theta_variance = 0.1;
    double range_variance = 0.2, bearing_variance = 0.1;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(prior_x, prior_y, prior_theta)); // 30cm std on x,y, 0.1 rad on theta
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(bearing_variance, range_variance)); // 0.1 rad std on bearing, 20cm on range
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(x_variance, y_variance, theta_variance)); // 20cm std on x,y, 0.1 rad on theta
    // create factor graph symbol for erobot.getPosition()very camera pose and landmark
    vector<Symbol> odometry_symbol;
    vector<Symbol> landmark_symbol;
    //camera pose at the initial position
    odometry_symbol.push_back(Symbol('c',0));
    //odometry start from c1
    for(int i=1;i<=odometry.size();i++){
        odometry_symbol.push_back(Symbol('c',i));
        odometry_symbol[i].print("odometry symbol: ");
    }
    //landmark start from c0
    for(int i=0;i<num_landmark;i++){
        landmark_symbol.push_back(Symbol('l',i));
        landmark_symbol[i].print("landmark symbol: ");
    }
    Values initialEstimate;
    initialEstimate.insert(odometry_symbol[0], robot.getPosition());
    odom_x = robot.getPosition().x();
    odom_y = robot.getPosition().y();
    odom_theta = robot.getPosition().theta();
    //create the prior with the robot initial position
    graph.add(PriorFactor<Pose2>(odometry_symbol[0], robot.getPosition(), priorNoise));
    vector<int> detected = detect_landmark(robot);
    //testing: what are the landmarks detected
    for(int i = 0; i < detected.size();i++){
        cout << detected[i] << endl;
    }
    if(!detected.empty()){
        for(int i = 0 ; i < detected.size();i++){
            graph.add(BearingRangeFactor<Pose2, Point2>(odometry_symbol[0], landmark_symbol[detected[i]], robot.getPosition().bearing(landmark[detected[i]]), robot.getPosition().range(landmark[detected[i]]), measurementNoise));
            //use the absolute coordinate of landmarks, later can change to the converted version from measurement.
            initialEstimate.insert(landmark_symbol[detected[i]], landmark[detected[i]]);
        }
    }
    //robot start moving
    int num_odometry = odometry.size();
    for(int i = 1 ; i <= num_odometry;i++){
        Pose2 delta_position = odometry[i-1];
        Pose2 odom_position(odom_x,odom_y,odom_theta);
        Pose2 old_position = robot.getPosition();
        initialEstimate.insert(odometry_symbol[i], odom_position.compose(delta_position));
        odom_x = odom_position.compose(delta_position).x();
        odom_y = odom_position.compose(delta_position).y();
        odom_theta = odom_position.compose(delta_position).theta();
        //include random noise to the next robot position
        robot.setPosition(old_position.compose(delta_position).x()+addNoise(x_variance), old_position.compose(delta_position).y()+addNoise(y_variance),old_position.compose(delta_position).theta()+addNoise((theta_variance)));
        robot.getPosition().print("\nrobot position ground truth: ");
        vector<int> detected = detect_landmark(robot);
        if(!detected.empty()) {
            for (int j = 0; j < detected.size(); j++) {
                graph.add(BearingRangeFactor<Pose2, Point2>(odometry_symbol[i], landmark_symbol[detected[j]],
                                                            robot.getPosition().bearing(landmark[detected[j]]),
                                                            robot.getPosition().range(landmark[detected[j]]),
                                                            measurementNoise));
                if (!initialEstimate.exists(landmark_symbol[detected[j]])) {
                    initialEstimate.insert(landmark_symbol[detected[j]], landmark[detected[j]]);
                }
            }
        }
        graph.add(BetweenFactor<Pose2>(odometry_symbol[i-1], odometry_symbol[i], odometry[i-1], odometryNoise));
    }
    graph.print("\ngraph in the final move: \n");
    initialEstimate.print("\ninitial estimate: \n");
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("\nFinal Result:\n");
}



