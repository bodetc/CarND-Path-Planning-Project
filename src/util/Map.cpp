//
// Created by Cédric Bodet on 28.10.17.
//

#include <cmath>
#include "Map.h"
#include "../utils.h"

using namespace std;

Map::Map(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    maps_x(maps_x), maps_y(maps_y), maps_s(maps_s) {

  spline_x.set_points(maps_s, maps_x);
  spline_y.set_points(maps_s, maps_y);
}

vector<double> Map::getFrenet(double x, double y, double theta) const {
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

vector<double> Map::getXY(double s, double d) const {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}


Trajectory Map::getXY(const Trajectory& sdTrajectory) const {
  Trajectory xyTrajectory;
  for (unsigned long i = 0; i < sdTrajectory.size(); i++) {
    auto sd = sdTrajectory.at(i);
    xyTrajectory.push_back(getXY(sd[0], sd[1]));
  }
  return xyTrajectory;
}

std::vector<double> Map::getXYspline(double s, double d) const {
  double x = spline_x(s);
  double y = spline_y(s);

  double heading = atan2(spline_y.deriv(1, s), spline_x.deriv(1, s));
  double perp_heading = heading - M_PI / 2;

  x += d * cos(perp_heading);
  y += d * sin(perp_heading);

  return {x, y};
}

Trajectory Map::getXYspline(const Trajectory &sdTrajectory) const {
  Trajectory xyTrajectory;
  for (unsigned long i = 0; i < sdTrajectory.size(); i++) {
    auto sd = sdTrajectory.at(i);
    auto xy = getXYspline(sd[0], sd[1]);
    xyTrajectory.push_back(xy);
  }
  return xyTrajectory;
}

int Map::ClosestWaypoint(double x, double y) const {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta) const {
  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > M_PI / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}
