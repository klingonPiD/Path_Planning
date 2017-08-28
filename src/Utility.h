//
//  Utility.hpp
//  Path_Planning
//
//  Created by Ajay on 8/23/17.
//
//

#ifndef Utility_h
#define Utility_h

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

using namespace::std;

class Utility {
public:
    constexpr static const double speedLimit = 49.0; //miles/hr
    constexpr static const double speed_delta = 0.112; //metre/s
    constexpr static const double lane_width = 4.0; //m
    constexpr static const double prediction_dist = 30.0;//m
    constexpr static const double s_thresh = 20.0;//m
    constexpr static const double delta_t = 0.02; //s
    static double deg2rad(double x);
    static double rad2deg(double x);
    static double distance(double x1, double y1, double x2, double y2);
    static int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    static int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
    
};

#endif /* Utility_hpp */
