#ifndef Car_hpp
#define Car_hpp

#include <stdio.h>
#include "Map.h"
#include "json.hpp"

class Car{
public:
  Car(nlohmann::json j);
  virtual ~Car();
  
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int current_lane;
  int num_path_points;
  double time_per_step;
  double max_acceleration;
  
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  void keepInLane(Map &map);
  bool isInLane(float d, int lane, double lane_width);
  double getVelocity(double ref_velocity);
  
private:
  void printVector(vector<double> vec){
    for (int i = 0; i<vec.size(); i++)
      cout << vec[i] << " ";
  }
  
  // For converting back and forth between radians and degrees.
  double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }
};

#endif /* Car_hpp */
