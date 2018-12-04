#include "Map.h"

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>

Map::Map() {}
Map::~Map() {}

Map::Map(string mapFile){
  max_s = 6945.554;
  lane_width = 4;
  speed_limit = 50 / 2.237; //mph to m/s
  
  ifstream in_map(mapFile.c_str(), ifstream::in);
  if (in_map.fail()){
    // the Xcode project executable is a folder deeper
    string oneLayerDeeper = "../";
    in_map = ifstream(oneLayerDeeper + mapFile.c_str(), ifstream::in);
  }
  
  string line;
  while (getline(in_map, line)) {
      istringstream iss(line);
      double x;
      double y;
      float s;
      float d_x;
      float d_y;
      iss >> x;
      iss >> y;
      iss >> s;
      iss >> d_x;
      iss >> d_y;
      waypoints_x.push_back(x);
      waypoints_y.push_back(y);
      waypoints_s.push_back(s);
      waypoints_dx.push_back(d_x);
      waypoints_dy.push_back(d_y);
  }
}

void Map::processSensorFusion(vector<vector<double>> sensor_fusion){
  otherCars = map<int, set<Car>>();
  for (vector<double> other_car : sensor_fusion){
    Car otherCar;
    otherCar.s = other_car[5];
    otherCar.d = other_car[6];
    otherCar.current_lane = otherCar.d / lane_width;
    
    double other_vx = other_car[3];
    double other_vy = other_car[4];
    otherCar.speed = sqrt(other_vx*other_vx + other_vy*other_vy);
    
    otherCars[otherCar.current_lane].insert(otherCar);
  }
}

bool Map::laneIsOpen(int lane, double s, double speed){
  if (lane < 0)
    return false;
  if (lane > 2)
    return false;
  
  set<Car> otherCarsInLane = otherCars[lane];
  for (Car car : otherCarsInLane){
    if (car.current_lane == lane){
      if (car.s >= s-5 && car.s < s+30){
        return false;
      }
      if (car.s > s-30 && car.s < s-5 && car.speed > speed){
        return false;
      }
    }
  }
  return true;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d)
{
    int prev_wp = -1;
    
    while(s > waypoints_s[prev_wp+1] && (prev_wp < (int)(waypoints_s.size()-1) ))
    {
        prev_wp++;
    }
    
    int wp2 = (prev_wp+1)%waypoints_x.size();
    
    double heading = atan2((waypoints_y[wp2]-waypoints_y[prev_wp]),(waypoints_x[wp2]-waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-waypoints_s[prev_wp]);
    
    double seg_x = waypoints_x[prev_wp]+seg_s*cos(heading);
    double seg_y = waypoints_y[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-M_PI/2;
    
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    
    return {x,y};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x,y, theta);
  
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = waypoints_x.size()-1;
  }
  
  double n_x = waypoints_x[next_wp]-waypoints_x[prev_wp];
  double n_y = waypoints_y[next_wp]-waypoints_y[prev_wp];
  double x_x = x - waypoints_x[prev_wp];
  double x_y = y - waypoints_y[prev_wp];
  
  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;
  
  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  
  //see if d value is positive or negative by comparing it to a center point
  
  double center_x = 1000-waypoints_x[prev_wp];
  double center_y = 2000-waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(waypoints_x[i],waypoints_y[i],waypoints_x[i+1],waypoints_y[i+1]);
  }
  
  frenet_s += distance(0,0,proj_x,proj_y);
  
  return {frenet_s,frenet_d};
}

double Map::distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::ClosestWaypoint(double x, double y)
{
  
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  
  for(int i = 0; i < waypoints_x.size(); i++)
  {
    double map_x = waypoints_x[i];
    double map_y = waypoints_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
    
  }
  return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta)
{
  
  int closestWaypoint = ClosestWaypoint(x,y);
  
  double map_x = waypoints_x[closestWaypoint];
  double map_y = waypoints_y[closestWaypoint];
  
  double heading = atan2((map_y-y),(map_x-x));
  
  double angle = fabs(theta-heading);
  angle = min(2*M_PI - angle, angle);
  
  if(angle > M_PI/4)
  {
    closestWaypoint++;
    if (closestWaypoint == waypoints_x.size())
    {
      closestWaypoint = 0;
    }
  }
  
  return closestWaypoint;
}
