#include "Car.hpp"
#include "spline.h"
#include "Map.h"

Car::Car(string mapFile) {
  map = new Map(mapFile);
  current_lane = 1;
  num_path_points = 25;
  time_per_step = .02;
  max_acceleration = 8.5;
  max_vel_change = max_acceleration * time_per_step;
}

Car::Car() {};
Car::~Car() {
  //clean up the map
}

void Car::readInputData(nlohmann::json j){
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
  if (temp_previous_path_x.size() > 2){
    // sometimes the simulator only returns a single point. When that happens just use the old previous path to avoid stopping.
    previous_path_x = temp_previous_path_x;
    previous_path_y = temp_previous_path_y;
  }
  
  map->processSensorFusion(j["sensor_fusion"]);
}

double Car::getDistance(double x1, double y1, double x2, double y2){
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

bool operator< (const Car &c1, const Car &c2){
  return c1.d < c2.d;
}

void Car::drive(){
  
  // cases to fix: if the car in front continually brakes I get too close: slow down if the car in front is closer than 20
  // if the cars behind me in the other lane are going slower than me then go ahead and move anyway
  // don't cut off cars moving too fast
  // why do I randomly exceed limits?
  
  
  set<Car> otherCarsInLane = map->otherCars[current_lane];
  for (Car other_car : otherCarsInLane){
      if (other_car.current_lane == current_lane){
        if (other_car.s > s && other_car.s < s + 30){
          if (map->laneIsOpen(current_lane-1, s, speed)){
            generateTrajectory(current_lane-1, other_car.speed);
            current_lane = current_lane-1;
          }
          else if (map->laneIsOpen(current_lane+1, s, speed)){
            generateTrajectory(current_lane+1, other_car.speed);
            current_lane = current_lane+1;
          }
          else {
            generateTrajectory(current_lane, other_car.speed);
          }
          return;
        }
    }
  }
  
  // no one in front of us, drive as fast as legally allowed
  generateTrajectory(current_lane, map->speed_limit - (0.5/2.237));
  
}

void Car::generateTrajectory(int lane, double target_speed){
  vector<double> spline_x;
  vector<double> spline_y;
  double ref_x;
  double ref_y;
  double ref_yaw;
  double ref_speed;
  
  int previous_path_size = previous_path_x.size();
  
  if (previous_path_size < 2){
    // we don't have a previously calculated path, so use the car's current position as the reference
    ref_x = x;
    ref_y = y;
    ref_yaw = deg2rad(yaw);
    ref_speed = speed;
    
    // use a point that is line with current car orientation
    spline_x.push_back(x - cos(ref_yaw));
    spline_y.push_back(y - sin(ref_yaw));
    
    // current car location
    spline_x.push_back(x);
    spline_y.push_back(y);
  }
  else{
    // use the end of the previous path as the reference, we will just add points from there
    ref_x = previous_path_x[previous_path_size-1];
    ref_y = previous_path_y[previous_path_size-1];
    
    double ref_x_prev = previous_path_x[previous_path_size-2];
    double ref_y_prev = previous_path_y[previous_path_size-2];
    
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    ref_speed = getDistance(ref_x, ref_y, ref_x_prev, ref_y_prev)/time_per_step;
    
    spline_x.push_back(ref_x_prev);
    spline_y.push_back(ref_y_prev);
    
    spline_x.push_back(ref_x);
    spline_y.push_back(ref_y);
  }
  
  // get new points after the reference points for the spline
  double distance_between_points = 30;
  vector<double> ref_s_d = map->getFrenet(ref_x, ref_y, ref_yaw);
  for (int i=1; i<4; i++){
    vector<double> next_spline_point = map->getXY(ref_s_d[0]+(distance_between_points*i), map->lane_width/2.0+(map->lane_width*lane));
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
  
  // follow the previous path, we will add to the end of it
  next_x_vals = previous_path_x;
  next_y_vals = previous_path_y;
  
  double current_step_speed = ref_speed;
  double current_step_x = 0;
  double current_step_y = 0;
  
  for (int i=1; i<num_path_points-previous_path_size; i++){
    // change speed on each time step otherwise we only accelerate every time we re-calculate a path which is 3 or 4 time steps.
    if (current_step_speed + max_vel_change < target_speed){
      current_step_speed += max_vel_change;
    }
    else if (current_step_speed - max_vel_change > target_speed){
      current_step_speed -= max_vel_change;
    }
    
    // figure out how far we can go given the current speed we are aiming for
    double target_x = current_step_x + 30.0;
    double target_y = spline(target_x);
    double target_distance = getDistance(target_x, target_y, current_step_x, current_step_y);
    double N = (target_distance/(time_per_step*current_step_speed));
    
    // get the next point
    double x_point = current_step_x + (target_x-current_step_x)/N;
    double y_point = spline(x_point);
    
    current_step_x = x_point;
    current_step_y = y_point;
    
    // convert the point back to global coordinates
    x_point = ref_x + (current_step_x * cos(ref_yaw) - current_step_y * sin(ref_yaw));
    y_point = ref_y + (current_step_x * sin(ref_yaw) + current_step_y * cos(ref_yaw));
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}
