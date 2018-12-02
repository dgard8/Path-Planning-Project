#ifndef Map_h
#define Map_h

#include <string>
#include <vector>
#include <math.h>

using namespace std;

class Map{
public:
  Map(string mapFile);
  
  virtual ~Map();
  
  double max_s;
  double lane_width;
  double speed_limit;
  
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
  vector<vector<double>> sensor_fusion;
  
  vector<double> getXY(double s, double d);
  vector<double> getFrenet(double x, double y, double theta);
  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
};

#endif /* Map_h */
