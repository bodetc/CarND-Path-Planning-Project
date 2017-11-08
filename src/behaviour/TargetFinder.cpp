//
// Created by Cédric Bodet on 08.11.17.
//

#include "TargetFinder.h"
#include "../definitions.h"

const Vehicle *
getNextInLane(const State &ego, int target_lane, double max_distance, const std::vector<Vehicle> &predictions) {
  ;
  const double target_d = (target_lane + .5) * LANE_WIDTH;

  double min_distance = std::numeric_limits<double>::max();
  const Vehicle *next_in_lane = NULL;
  for (const auto prediction : predictions) {
    const State &state = prediction.startState();
    double distance = state.s - ego.s;
    if (state.getLane() == target_lane // Is in the proper lane
        && distance > 0 // Is in front of us
        && distance < max_distance // Is not too far away
        && distance < min_distance // Is the closest so far
        ) {
      min_distance = distance;
      next_in_lane = &prediction;
    }
  }
  return next_in_lane;
}

const Vehicle TargetFinder::getTarget(const State &ego, int target_lane, double max_distance,
                                      const std::vector<Vehicle> &predictions) {

  const Vehicle *next_in_lane = getNextInLane(ego, target_lane, max_distance, predictions);

  // If nothing is found, make a virtual target in front of the car to follow
  if (next_in_lane == NULL) {
    State target_state{
        .s = ego.s + FOLLOW_DISTANCE,
        .s_dot = TARGET_SPEED,
        .d =  (target_lane + .5) * LANE_WIDTH,
    };
    return Vehicle(target_state);
  }
  return *next_in_lane;
}
