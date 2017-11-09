//
// Created by Cédric Bodet on 08.11.17.
//

#include <iostream>
#include "LaneSearch.h"
#include "../definitions.h"


int
LaneSearch::getInLane(const State &ego, int target_lane, double max_distance, const std::vector<Vehicle> &predictions,
                        bool forward) {
  double min_distance = std::numeric_limits<double>::max();
  int next_in_lane = -1;
  for (int i = 0; i < predictions.size(); i++) {
    auto prediction = predictions[i];
    const State &state = prediction.startState();
    double d = state.s - ego.s;
    double distance = fabs(d);
    if (state.getLane() == target_lane // Is in the proper lane
        && (forward ? d >= 0 : d <= 0) // Is in front/behind of us
        && distance < max_distance // Is not too far away
        && distance < min_distance // Is the closest so far
        ) {
      min_distance = distance;
      next_in_lane = i;
    }
  }
  return next_in_lane;
}

int LaneSearch::getNextInLane(const State &ego, int target_lane, double max_distance,
                                const std::vector<Vehicle> &predictions) {
  return getInLane(ego, target_lane, max_distance, predictions, true);
}

int LaneSearch::getPreviousInLane(const State &ego, int target_lane, double max_distance,
                                    const std::vector<Vehicle> &predictions) {
  return getInLane(ego, target_lane, max_distance, predictions, false);
}

const Vehicle LaneSearch::getTarget(const State &ego, int target_lane, double max_distance,
                                      const std::vector<Vehicle> &predictions) {
  State target_state{};

  const int next_in_lane = getNextInLane(ego, target_lane, max_distance, predictions);

  // If nothing is found, make a virtual target in front of the car to follow
  if (next_in_lane < 0) {
    target_state.s = ego.s + FOLLOW_DISTANCE;
    target_state.s_dot = ego.s_dot;
    target_state.s_ddot = std::min(0.9 * MAX_ACCEL, (TARGET_SPEED - ego.s_dot) / HORIZON);
  } else {
    std::cout << "Following car: ";
    target_state = predictions[next_in_lane].startState();
  }

  // Stay behind the car at a safe given distance

  // Slowly fill the gap to the car in front
  double current_distance = (target_state.s - ego.s);
  target_state.s -= .3 * FOLLOW_DISTANCE + .7 * current_distance;

  // Ensure that the target is not too far
  double final_distance = target_state.stateAt(HORIZON).s-ego.s;
  double max_speed_distance= TARGET_SPEED*HORIZON;
  if(final_distance>max_speed_distance) {
    target_state.s-=(final_distance-max_speed_distance);
  }

  // Do not reproduce the swerving of the car in front and stay in the middle lane
  target_state.d = (target_lane + .5) * LANE_WIDTH;
  target_state.d_dot = 0;
  target_state.d_ddot = 0;

  return Vehicle(target_state);
}
