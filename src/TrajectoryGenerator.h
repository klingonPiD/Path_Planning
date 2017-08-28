//
//  TrajectoryGenerator.hpp
//  Path_Planning
//
//  Created by Ajay on 8/23/17.
//
//

#ifndef TrajectoryGenerator_h
#define TrajectoryGenerator_h

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
#include "spline.h"
#include "BehaviorPlanner.h"

using namespace std;

class TrajectoryGenerator {
public:
    struct trajectoryData{
        // localization data
        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;
        // Previous path's end s and d values
        double end_path_s;
        double end_path_d;
        // lane info
        int lane;
    };
    struct anchorPoints{
        vector<double> anchor_ptsx;
        vector<double> anchor_ptsy;
    };
    
    
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    tk::spline anchor_spline;
    double ref_x;
    double ref_y;
    double ref_yaw;
    
    anchorPoints generateAnchorPoints(const vector<double> &previous_path_x, const vector<double> &previous_path_y, trajectoryData locData, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
    
    void fitSplineToAnchorPoints(anchorPoints anchor_pts, trajectoryData trajData);
    void generatePath(BehaviorPlanner::CarState egoState, double &target_v, double carAheadVelocity);
    
};



#endif /* TrajectoryGenerator_h */
