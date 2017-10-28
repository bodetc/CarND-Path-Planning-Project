//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H


#include <vector>
#include "Trajectory.h"
#include "../tk-spline/spline.h"

class Map {
private:
  const std::vector<double> maps_x;
  const std::vector<double> maps_y;
  const std::vector<double> maps_s;

  tk::spline spline_x;
  tk::spline spline_y;

public:
  Map(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta) const;

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d) const;

  Trajectory getXY(const Trajectory &sdTrajectory) const;

  // Transform from Frenet s,d coordinates to Cartesian x,y using the spline approximation
  std::vector<double> getXYspline(double s, double d) const;

  Trajectory getXYspline(const Trajectory &sdTrajectory) const;

private:
  int ClosestWaypoint(double x, double y) const;

  int NextWaypoint(double x, double y, double theta) const;
};


#endif //PATH_PLANNING_MAP_H
