//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H


#include <vector>
#include "definitions.h"
#include "Trajectory.h"

class Map {
private:
  const std::vector<double> maps_x;
  const std::vector<double> maps_y;
  const std::vector<double> maps_s;

public:
  Map(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  static inline bool is_in_lane(double d, int lane) {
    return d > LANE_WIDTH * lane && d < LANE_WIDTH * (lane + 1);
  }

  static inline double get_mid_lane(int lane) {
    return LANE_WIDTH * (lane + .5);
  }


  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta) const;

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d) const;
private:

  int ClosestWaypoint(double x, double y) const;

  int NextWaypoint(double x, double y, double theta) const;
};


#endif //PATH_PLANNING_MAP_H
