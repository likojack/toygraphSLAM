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

void Simulator::begin_simulate(int *) {

}


