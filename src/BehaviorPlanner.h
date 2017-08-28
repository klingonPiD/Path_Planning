//
//  BehaviorPlanner.h
//  Path_Planning
//
//  Created by Ajay on 8/23/17.
//
//

#ifndef BehaviorPlanner_h
#define BehaviorPlanner_h

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "Utility.h"

using namespace std;

class BehaviorPlanner {
private:
    map<int, vector<vector<double>>> groupFilteredCarsByLane(vector<vector<double>> filteredCars, int lane, double car_s, int prev_size);
public:
    double carAheadVelocity;
    enum CarState {KL, KLS, LCL, LCR};
    CarState egoState = KL;
    bool checkCollision();
    vector<vector<double>> filterCars(vector<vector<double>> sensor_fusion, double car_s, double car_d);
    CarState determineLaneState(vector<vector<double>> filteredCars, double car_s, double car_d, int lane, int prev_size);
    
};



#endif /* BehaviorPlanner_h */
