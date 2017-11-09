//
// Created by Cédric Bodet on 27.10.17.
//

#include "Controller.h"
#include "utils.h"
#include "spline.h"

using namespace std;

Controller::Controller(const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s) :
    map(maps_x, maps_y, maps_s) {}

const Trajectory
Controller::compute_trajectory(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                               const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y,
                               double end_path_s, double end_path_d,
                               const SensorFusion &sensor_fusion) {
  int prev_size = previous_path_x.size();
  double lag = (double) prev_size * TIMESTEP;

  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = sensor_fusion.is_too_close(lag, lane, car_s);

  if (too_close) {
    ref_vel -= SPEED_STEP;
  } else if (ref_vel < TARGET_SPEED) {
    ref_vel += SPEED_STEP;
  }

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

  for(int i = 1; i <= N_SPLINE_FORWARD; i++) {
    vector<double> next_wp = map.getXY(car_s + i*SPLINE_DISTANCE, Map::get_mid_lane(lane));
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

  Trajectory trajectory (previous_path_x, previous_path_y);

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


