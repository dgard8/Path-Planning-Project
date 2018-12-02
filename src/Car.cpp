#include "Car.hpp"
#include "spline.h"

Car::~Car() {}

Car::Car(nlohmann::json j){
  x = j["x"];
  y = j["y"];
  s = j["s"];
  d = j["d"];
  yaw = j["yaw"];
  speed = j["speed"];
  speed /=  2.237; //mph to m/s
  end_path_s = j["end_path_s"];
  end_path_d = j["end_path_d"];
  
  vector<double> temp_previous_path_x = j["previous_path_x"];
  vector<double> temp_previous_path_y = j["previous_path_y"];
  previous_path_x = temp_previous_path_x;
  previous_path_y = temp_previous_path_y;
  
  current_lane = 1;
  num_path_points = 25;
  time_per_step = .02;
  max_acceleration = 9.5;
}

bool Car::isInLane(float d, int lane, double lane_width){
  double left_edge = lane_width*lane;
  double right_edge = left_edge + lane_width;
  return (d > left_edge && d < right_edge);
}

double Car::getVelocity(double target_velocity){
  double new_velocity;
  double max_vel_change = max_acceleration * time_per_step;
  if (speed + max_vel_change > target_velocity){
    if (speed - max_vel_change > target_velocity){
      new_velocity = speed - max_vel_change;
    }
    else{
      new_velocity = target_velocity;
    }
  }
  else{
    new_velocity = speed + max_vel_change;
  }
  cout << speed << " "  << new_velocity << endl;
  return new_velocity;
}

void Car::keepInLane(Map &map){
  
  vector<double> spline_x;
  vector<double> spline_y;
  double ref_x;
  double ref_y;
  double ref_yaw;
  
  double velocity = getVelocity(map.speed_limit - 0.5);
  
  for (vector<double> other_car : map.sensor_fusion){
    float other_car_d = other_car[6];
    if (isInLane(other_car_d, current_lane, map.lane_width)){
      double other_vx = other_car[3];
      double other_vy = other_car[4];
      double other_speed = sqrt(other_vx*other_vx + other_vy*other_vy);
      double other_s = other_car[5];
      
      if (other_s > s && other_s < s + 30){
        velocity = getVelocity(other_speed);
      }
    }
  }
  
  int previous_path_size = previous_path_x.size();
  
  if (previous_path_size < 2){
    ref_x = x;
    ref_y = y;
    ref_yaw = deg2rad(yaw);
    
    // use a point that is line with current car orientation
    spline_x.push_back(x - cos(ref_yaw));
    spline_y.push_back(y - sin(ref_yaw));
    
    // current car location
    spline_x.push_back(x);
    spline_y.push_back(y);
  }
  else{
    ref_x = previous_path_x[previous_path_size-1];
    ref_y = previous_path_y[previous_path_size-1];
    
    double ref_x_prev = previous_path_x[previous_path_size-2];
    double ref_y_prev = previous_path_y[previous_path_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
    spline_x.push_back(ref_x_prev);
    spline_y.push_back(ref_y_prev);
    
    spline_x.push_back(ref_x);
    spline_y.push_back(ref_y);
  }
  
  // get points for the spline
  double distance_between_points = 30;
  for (int i=1; i<4; i++){
    vector<double> next_spline_point =
        map.getXY(s+(distance_between_points*i), map.lane_width/2.0+(map.lane_width*current_lane));
    spline_x.push_back(next_spline_point[0]);
    spline_y.push_back(next_spline_point[1]);
  }
  
  //shift points to local coordinates
  for (int i=0; i<spline_x.size(); i++){
    double shift_x = spline_x[i] - ref_x;
    double shift_y = spline_y[i] - ref_y;
    
    spline_x[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
    spline_y[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
  }
  
  tk::spline spline;
  spline.set_points(spline_x, spline_y);
  
  next_x_vals = previous_path_x;
  next_y_vals = previous_path_y;
  
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_distance = sqrt((target_x*target_x)+(target_y*target_y));
  double N = (target_distance/(time_per_step*velocity));
  //cout << N << endl;
  
  for (int i=1; i<num_path_points-previous_path_size; i++){
    double x_point = i*target_x/N;
    double y_point = spline(x_point);
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    
    //cout << " " << i << " " << x_point << " " << y_point << endl;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}
