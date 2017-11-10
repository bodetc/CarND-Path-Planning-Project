//
// Created by Cédric Bodet on 28.10.17.
//

#include <cmath>
#include "Map.h"
#include "utils.h"

using namespace std;

Map::Map(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    maps_x(maps_x), maps_y(maps_y), maps_s(maps_s) {
}

const Trajectory Map::complete_trajectory(int target_lane, double ref_vel,
                                          double car_x, double car_y, double car_s, double car_yaw,
                                          const std::vector<double> &previous_path_x,
                                          const std::vector<double> &previous_path_y) const {
  unsigned long prev_size = previous_path_x.size();

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  for (int i = 1; i <= N_SPLINE_FORWARD; i++) {
    // Setting d to the target lane in 30 meters and later allows for a smooth lane change once splined
    vector<double> next_wp = getXY(car_s + i * SPLINE_DISTANCE, Map::get_mid_lane(target_lane));
    ptsy.push_back(next_wp[1]);
    ptsx.push_back(next_wp[0]);
  }

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  tk::spline s;

  s.set_points(ptsx, ptsy);


  vector<double> next_x_vals;
  vector<double> next_y_vals;

  Trajectory trajectory(previous_path_x, previous_path_y);

  double target_x = SPLINE_DISTANCE;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  for (int i = 1; i <= N_STEPS - prev_size; i++) {

    double N = target_dist / (ref_vel * TIMESTEP);
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;


    trajectory.push_back(x_point, y_point);
  }

  return trajectory;
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
