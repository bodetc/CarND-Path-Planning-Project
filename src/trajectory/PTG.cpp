//
// Created by Cédric Bodet on 04.11.17.
//

#include "PTG.h"

using namespace std;

struct Goal {
  double s, d, t;
};

Trajectory PTG::operator()(State start_state, int target_vehicle, const State &delta, double T,
                           const vector<Vehicle> &predictions) {
  vector<Goal> allGoals;

  double timestep = 0.5;


  return Trajectory();
}
