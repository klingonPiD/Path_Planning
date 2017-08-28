//
//  TrajectoryGenerator.cpp
//  Path_Planning
//
//  Created by Ajay on 8/23/17.
//
//


#include "TrajectoryGenerator.h"
#include "Utility.h"


//Trajectory generation logic
//1. Create anchor points and fit a spline
//   - Anchor points contain two prev points, current point and 2 future points
//2. Transform anchor pts (translation and rotation) to make curr car pos at origin
//3. Fill next car pos with previous car pos (chaining previous path. For the remaining points (total len 50) in next car pos add predicted points computed using the anchor spline.


TrajectoryGenerator::anchorPoints TrajectoryGenerator::generateAnchorPoints(const vector<double> &previous_path_x, const vector<double> &previous_path_y, trajectoryData trajData, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_size = previous_path_x.size();
    TrajectoryGenerator::anchorPoints anchor_pts;
    double anchor_theta = Utility::deg2rad(trajData.car_yaw);
    
    // Throw in some prev points
    double prev_x1, prev_y1, prev_x2, prev_y2;
    if(prev_size < 2)
    {
        prev_x1 = trajData.car_x;
        prev_y1 = trajData.car_y;
        prev_x2 = trajData.car_x - cos(anchor_theta);//anchor_theta
        prev_y2 = trajData.car_y - sin(anchor_theta);//anchor_theta
    }
    else
    {
        prev_x1 = previous_path_x[prev_size - 1];
        prev_x2 = previous_path_x[prev_size - 2];
        prev_y1 = previous_path_y[prev_size - 1];
        prev_y2 = previous_path_y[prev_size - 2];
        anchor_theta = atan2(prev_y1 - prev_y2, prev_x1 - prev_x2);
    }
    
    anchor_pts.anchor_ptsx.push_back(prev_x2);
    anchor_pts.anchor_ptsx.push_back(prev_x1);
    anchor_pts.anchor_ptsy.push_back(prev_y2);
    anchor_pts.anchor_ptsy.push_back(prev_y1);
    
    double temp = (prev_size > 0)? trajData.end_path_s : trajData.car_s;
    //throw in some future pts
    for(int i=1; i<4; i++)
    {
        double future_s = temp + i * Utility::prediction_dist; // car_s
        double future_d = trajData.lane * Utility::lane_width + Utility::lane_width/2.0;
        vector<double> xy = Utility::getXY(future_s, future_d, maps_s, maps_x, maps_y);
        anchor_pts.anchor_ptsx.push_back(xy[0]);
        anchor_pts.anchor_ptsy.push_back(xy[1]);
        
    }
    return anchor_pts;
    
}

void TrajectoryGenerator::fitSplineToAnchorPoints(TrajectoryGenerator::anchorPoints anchor_pts, trajectoryData trajData)
{
    //These are required references
    int prev_size = previous_path_x.size();
    ref_x = trajData.car_x;
    ref_y = trajData.car_y;
    //double ref_yaw = deg2rad(car_yaw);
    if(prev_size >= 2)
    {
        double prev_x1 = previous_path_x[prev_size - 1];
        double prev_y1 = previous_path_y[prev_size - 1];
        ref_x = prev_x1;
        ref_y = prev_y1;
    }
    
    // center the anchor pts with respect to the currrent car pos
    // A +ve steering angle in the simulator implies a -ve angle in the
    // vehicle model and vice versa
    ref_yaw = Utility::deg2rad(trajData.car_yaw);
    double psi = -ref_yaw;
    for(int i=0; i < anchor_pts.anchor_ptsx.size();i++)
    {
        // Note: Subtracting px and py from the way points has the effect
        // of centering the vehicle at origin. Makes the math easier.
        double trans_x = anchor_pts.anchor_ptsx[i] - ref_x;
        double trans_y = anchor_pts.anchor_ptsy[i] - ref_y;
        // Applying rotation transform [cos(theta) -sin(theta); sin(theta) cos(theta)]
        // Translation not required since vehicle is now centered at origin
        anchor_pts.anchor_ptsx[i] = cos(psi)*trans_x - sin(psi)*trans_y;
        anchor_pts.anchor_ptsy[i] = sin(psi)*trans_x + cos(psi)*trans_y;
    }
    
    
    //fit spline to anchor pts
    anchor_spline.set_points(anchor_pts.anchor_ptsx, anchor_pts.anchor_ptsy);
    
}


void TrajectoryGenerator::generatePath(BehaviorPlanner::CarState egoState, double &target_v, double carAheadVelocity)
{
    for(int i=0;i<previous_path_x.size();i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    
    int total_pred_len = 50;
    double target_x = Utility::prediction_dist;//30.0; //metres
    double target_y = anchor_spline(target_x);
    double target_d = sqrt(target_x * target_x + target_y * target_y);
    double ref_v = Utility::speedLimit * 0.447; //mph to metres/s
    if(egoState == BehaviorPlanner::CarState::KLS)
    {
        if(target_v < carAheadVelocity)
        {
            target_v += Utility::speed_delta;
        }
        else
        {
            target_v -= Utility::speed_delta;
        }
    }
    else
    {
        if(target_v < ref_v)// && !filteredCars.size())
        {
            target_v += Utility::speed_delta;
        }
        else
        {
            target_v -= Utility::speed_delta;
        }
    }
    //target_v = ref_v;
    
    double num_points =  target_d/(Utility::delta_t * target_v);
    //std::cout << "num points " << num_points << std::endl;
    double x_inc = 0.0;
    double psi = ref_yaw;
    for(int i=0;i< total_pred_len-previous_path_x.size();i++)
    {
        x_inc += (target_x / num_points);
        double y_inc = anchor_spline(x_inc);
        
        //todo: describe this
        double x_future = cos(psi)*x_inc - sin(psi)*y_inc;
        double y_future = sin(psi)*x_inc + cos(psi)*y_inc;
        
        x_future += ref_x;
        y_future += ref_y;
        
        next_x_vals.push_back(x_future);
        next_y_vals.push_back(y_future);
        
    }
    
    
    
}



