//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H


#include <vector>

class Map {
private:
  const std::vector<double> maps_x;
  const std::vector<double> maps_y;
  const std::vector<double> maps_s;

public:
  Map(const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d);

private:
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
};


#endif //PATH_PLANNING_MAP_H
