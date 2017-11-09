//
// Created by Cédric Bodet on 09.11.17.
//

#include <iostream>
#include "BehaviourPlanner.h"
#include "LaneSearch.h"

const Vehicle BehaviourPlanner::update(const State &ego, const std::vector<Vehicle> &predictions) {
  updateTargetLane(ego, predictions);

  double T = HORIZON;
  double max_distance = TARGET_SPEED * HORIZON;

  Vehicle target = LaneSearch::getTarget(ego, target_lane, max_distance, predictions);
  State goal = target.stateAt(T);
  std::cout << "Goal=(" << goal.s << "," << goal.d << ")" << std::endl;

  return target;
}

void BehaviourPlanner::updateTargetLane(const State &ego, const std::vector<Vehicle> &predictions) {
  if (last_lane_check < N_STEPS_LANE_CHECK) {
    last_lane_check++;
    return;
  }

  std::cout << "Performing lane check" << std::endl;
  last_lane_check = 0;

  // Check if it is possible to go to the right lane
  if (checkRightLane(ego, predictions)) {
    target_lane++;
    // If not, then check if it is possible/needed to pass
  } else if (checkLeftLane(ego, predictions)) {
    target_lane--;
  }
}

bool BehaviourPlanner::checkRightLane(const State &ego, const std::vector<Vehicle> &predictions) {
  int new_target = target_lane + 1;
  return new_target < N_LANES // Check that there is a lane to go to
         // Check that there is a car in front that needs overtaking
         && shouldOvertake(ego, predictions)
         // And no car in front in the lane
         && LaneSearch::getNextInLane(ego, new_target, RIGHT_LANE_CHECK_FRONT, predictions) == -1
         // And no car behind in the lane
         && LaneSearch::getPreviousInLane(ego, new_target, RIGHT_LANE_CHECK_BACK, predictions) == -1;
}

bool BehaviourPlanner::checkLeftLane(const State &ego, const std::vector<Vehicle> &predictions) {
  int new_target = target_lane - 1;
  return new_target >= 0 // Check that there is a lane to go to
         // Check that there is a car in front that needs overtaking
         && shouldOvertake(ego, predictions)
         // And no car in front in the lane
         && LaneSearch::getNextInLane(ego, new_target, LEFT_LANE_CHECK_FRONT, predictions) == -1
         // And no car behind in the lane
         && LaneSearch::getPreviousInLane(ego, new_target, LEFT_LANE_CHECK_BACK, predictions) == -1;
}

bool BehaviourPlanner::shouldOvertake(const State &ego, const std::vector<Vehicle> &predictions) {
  int next_id = LaneSearch::getNextInLane(ego, target_lane, PASSING_DISTANCE, predictions);
  if(next_id<0) {
    return false;
  }
  double speed = predictions[next_id].startState().d_dot;
  return speed<PASSING_SPEED;
}
