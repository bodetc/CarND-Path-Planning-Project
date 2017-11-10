//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H


#include <vector>
#include "definitions.h"
#include "Trajectory.h"
#include "spline.h"

class Map {
private:
  const std::vector<double> maps_x;
  const std::vector<double> maps_y;
  const std::vector<double> maps_s;

public:
  Map(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  // Append trajectory using the new reference velocity and target lane
  const Trajectory complete_trajectory(int target_lane, double ref_vel,
                                       double car_x, double car_y, double car_s, double car_yaw,
                                       const std::vector<double> &previous_path_x,
                                       const std::vector<double> &previous_path_y) const;

  // Returns true if d is within the given lane boundaries
  static inline bool is_in_lane(double d, int lane) {
    return d > LANE_WIDTH * lane && d < LANE_WIDTH * (lane + 1);
  }

  // Return the Frenet d coordinate of the middle point of the lane
  static inline double get_mid_lane(int lane) {
    return LANE_WIDTH * (lane + .5);
  }

private:

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d) const;

};


#endif //PATH_PLANNING_MAP_H
