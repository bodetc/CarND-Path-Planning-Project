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

constexpr bool DEBUG = true; // Set to true to get control logic log messages
constexpr bool VERBOSE = false; // Set to true to get much mor log messages

////////////////////////////////
// Physical world definitions //
////////////////////////////////

constexpr double MPH_TO_METERS_PER_SECOND = 0.44704;

// Target driving speed
constexpr double TARGET_MPH = 49.5;
constexpr double TARGET_SPEED = TARGET_MPH*MPH_TO_METERS_PER_SECOND;

// Speed under with to look for passing opportunity
constexpr double PASSING_MPH = 45.;
constexpr double PASSING_SPEED = PASSING_MPH*MPH_TO_METERS_PER_SECOND;

// Characteristics of the road
constexpr double LANE_WIDTH = 4; // m
constexpr double N_LANES = 3;

//////////////////////////
// Control definitions //
/////////////////////////

constexpr int N_SPLINE_FORWARD = 3; // How many points forward to spline
constexpr double SPLINE_DISTANCE = 30; // How far to set the points

constexpr double FORWARD_DISTANCE = 30; // m. Distance under which to slow down
constexpr double SPEED_STEP = 0.1; // m/s -> a = 2 m/s

// Distance front/back in the neighbouring lane which must be free for a lane change
constexpr double LANE_CHECK_FORWARD_DISTANCE = 40; // m
constexpr double LANE_CHECK_BACKWARDS_DISTANCE = 20; // m


#endif //PATH_PLANNING_DEFINITIONS_H_H
