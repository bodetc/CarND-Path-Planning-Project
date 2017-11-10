//
// Created by Cédric Bodet on 09.11.17.
//

#ifndef PATH_PLANNING_DEFINITIONS_H_H
#define PATH_PLANNING_DEFINITIONS_H_H

////////////////////////////
// Simulation definitions //
////////////////////////////

constexpr double TIMESTEP = .02; // s
constexpr int N_STEPS = 50;
constexpr bool DEBUG = true;

////////////////////////////////
// Physical world definitions //
////////////////////////////////

constexpr double MPH_TO_METERS_PER_SECOND = 0.44704;
constexpr double TARGET_MPH = 49.5;
constexpr double PASSING_MPH = 45.;

constexpr double TARGET_SPEED = TARGET_MPH*MPH_TO_METERS_PER_SECOND;
constexpr double PASSING_SPEED = PASSING_MPH*MPH_TO_METERS_PER_SECOND;

constexpr double LANE_WIDTH = 4;
constexpr double N_LANES = 3;

//////////////////////////
// Control definitions //
/////////////////////////

constexpr int N_SPLINE_FORWARD = 3;
constexpr double SPLINE_DISTANCE = 30;

constexpr double FORWARD_DISTANCE = 30;
constexpr double SPEED_STEP = 0.1;

constexpr double LANE_CHECK_FORWARD_DISTANCE = 30;
constexpr double LANE_CHECK_BACKWARDS_DISTANCE = 25;


#endif //PATH_PLANNING_DEFINITIONS_H_H
