//
// Created by kejie on 28/03/17.
//

#ifndef TOYGRAPHSLAM_SIMULATOR_H
#define TOYGRAPHSLAM_SIMULATOR_H

class Simulator {
//simulator should generate a map with the size of map and the number of landmark given
	int num_landmark;
	int width;
	int length;
  public:
	Simulator (int num_mark,int wi,int len) : num_landmark(num_mark), width(wi), length(len) {}
    int** generate_landmark(); //return a set of landmark position, an array of Point2
	void begin_simulate(int* ); // given robot initial position, a series of commands, and the landmark positions

};


#endif //TOYGRAPHSLAM_SIMULATOR_H
