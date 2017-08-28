//
//  BehaviorPlanner.cpp
//  Path_Planning
//
//  Created by Ajay on 8/23/17.
//
//

#include "BehaviorPlanner.h"
using namespace std;

bool BehaviorPlanner::checkCollision()
{
    return false;
}

vector<vector<double> > BehaviorPlanner::filterCars(vector<vector<double>> sensor_fusion, double car_s, double car_d)
{
    vector<vector<double>> filtered_cars;
    
    //cycle through sensor fusion data
    
    for(int i=0; i<sensor_fusion.size();i++ )
    {
        auto adj_car_s = sensor_fusion[i][5];
        //auto adj_car_d = sensor_fusion[i][6];
        
        bool s_flag = abs(car_s - adj_car_s) < Utility::s_thresh;//Utility::prediction_dist;
        //bool d_flag = abs(car_d - adj_car_d) < 2.0;
        
        if(s_flag)// && d_flag)
        {
            filtered_cars.push_back(sensor_fusion[i]);
        }
        
    }
    
    
    return filtered_cars;
}

BehaviorPlanner::CarState BehaviorPlanner::determineLaneState(vector<vector<double>> filteredCars, double car_s, double car_d, int lane, int prev_size)
{
    set<CarState> states = {KL, KLS, LCL, LCR}; //use set
    if(lane == 0)
    {
        states.erase(LCL);
    }
    if(lane == 2)
    {
        states.erase(LCR);
        
    }
    
    double ref_v = Utility::speedLimit * 0.447;// to m/s
    this->carAheadVelocity = ref_v;
    
    //1. Group the filtered cars according to lanes
    auto laneMap = groupFilteredCarsByLane(filteredCars, lane, car_s, prev_size);
    
    //2. If cars exist in same lane, move car to one of the empty ADJACENT lanes
    if(!laneMap.empty() && laneMap.find(lane) != laneMap.end())
    {
        //cars exist on the current lane
        //By definition all cars in ego lane in filtered list are ahead of ego car
        //execute lane change behavior if posible
        //first preference - move to left lane
        if(states.find(LCL) != states.end())
        {
            //if no filtered cars exist in left lane
            if(laneMap.find(lane - 1) == laneMap.end())
            {
                this->egoState = LCL;
                return LCL;
            }
            
        }
        //second preference - move to right lane
        if(states.find(LCR) != states.end())
        {
            //if no filtered cars exist in right lane
            if(laneMap.find(lane + 1) == laneMap.end())
            {
                this->egoState = LCR;
                return LCR;
            }
            
        }
        //if lane change not possible, keep following forward car at its speed
        std::cout << "KLS area " << std::endl;
        auto sameLaneCars = laneMap[lane];
        double min_s = 1000.0;
        std::cout << "KLS area: num cars in same lane " << sameLaneCars.size() << std::endl;
        
        if(sameLaneCars.size() == 1)
        {
            // there is only 1 car ahead. Just follow it at its speed
            auto ahead_car_x_vel = sameLaneCars[0][3];
            auto ahead_car_y_vel = sameLaneCars[0][4];
            double ahead_car_vel = sqrt(ahead_car_x_vel * ahead_car_x_vel + ahead_car_y_vel * ahead_car_y_vel);
            std::cout << "KLS area: adj car vel " << ahead_car_vel << std::endl;
            this->carAheadVelocity = (ahead_car_vel < ref_v) ? ahead_car_vel: ref_v;
        }
        else
        {
            // there is more than 1 car ahead. Find the car with min s and follow it
            for(int i=0; i<sameLaneCars.size();i++)
            {
                
                auto ahead_car_s = sameLaneCars[i][5];
                auto ahead_car_x_vel = sameLaneCars[i][3];
                auto ahead_car_y_vel = sameLaneCars[i][4];
                double ahead_car_vel = sqrt(ahead_car_x_vel * ahead_car_x_vel + ahead_car_y_vel * ahead_car_y_vel);// * 2.24;
                //simple linear predictor to determine future s
                ahead_car_s += (ahead_car_vel * Utility::delta_t * prev_size);
                
                if(ahead_car_s < min_s)
                {
                    std::cout << "KLS area: adj car vel " << ahead_car_vel << std::endl;
                    min_s = ahead_car_s;
                    this->carAheadVelocity = (ahead_car_vel < ref_v) ? ahead_car_vel: ref_v;
                }
                
            }
        }
        std::cout << "KLS area vlocity " << this->carAheadVelocity << std::endl;
        this->egoState = KLS;
        return KLS;
        
    }
    else
    {
        //no cars ahead - keep lane
        this->egoState = KL;
        return KL;
    }
    
}

map<int, vector<vector<double>>> BehaviorPlanner::groupFilteredCarsByLane(vector<vector<double>> filteredCars, int lane, double car_s, int prev_size)
{
    map<int, vector<vector<double>>> laneMap;
    for(int i=0; i<filteredCars.size();i++ )
    {
        auto adj_car_s = filteredCars[i][5];
        auto adj_car_d = filteredCars[i][6];
        int adj_car_lane = (int)(adj_car_d/4.0);
        
        // simple linear predictor to predict future s pos for adj car
        // this is done because you want to group the cars based on future pos rather than current snapshot
        // For eg. A fast moving car might be behind ego car in current snapshot but might be ahead in the next instant
        auto adj_car_x_vel = filteredCars[i][3];
        auto adj_car_y_vel = filteredCars[i][4];
        double adj_car_vel = sqrt(adj_car_x_vel * adj_car_x_vel + adj_car_y_vel * adj_car_y_vel);
        adj_car_s += (adj_car_vel * Utility::delta_t * prev_size);
        
        if(laneMap.find(adj_car_lane) == laneMap.end())//key does not exist
        {
            //if filtered car in same lane then include only those ahead of ego car
            if(adj_car_lane == lane)
            {
                if(adj_car_s >= car_s)
                    laneMap[adj_car_lane] = {filteredCars[i]};
            }
            else
            {
                laneMap[adj_car_lane] = {filteredCars[i]};
            }
        }
        else //key exists
        {
            auto adj_car_vec = laneMap[adj_car_lane];
            if(adj_car_lane == lane)
            {
                if(adj_car_s >= car_s)
                    adj_car_vec.push_back(filteredCars[i]);
            }
            else
            {
                adj_car_vec.push_back(filteredCars[i]);
            }
        }
        
    }
    return laneMap;
}

