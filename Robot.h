//
// Created by kejie on 29/03/17.
//

#ifndef TOYGRAPHSLAM_ROBOT_H
#define TOYGRAPHSLAM_ROBOT_H


class Robot {
    float detect_range;
    float detect_angle;
    float x;
    float y;
    float theta;
    float variance;
public:
    Robot(float detect_range, float detect_angle, float variance) : detect_range(detect_range), detect_angle(detect_angle), variance(variance) {}
    Robot(float detect_range, float detect_angle, float x, float y, float theta) : detect_range(detect_range),
                                                                                          detect_angle(detect_angle), x(x),
                                                                                          y(y), theta(theta) {}
    float report_position ();

    float move(); // input a command (r,d), output a position and pose (x,y,theta) with noise

};




#endif //TOYGRAPHSLAM_ROBOT_H
