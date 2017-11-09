//
// Created by Cédric Bodet on 28.10.17.
//

#include <profile.h>

#ifndef PATH_PLANNING_DEFINITIONS_H
#define PATH_PLANNING_DEFINITIONS_H

////////////////////////////
// Simulation definitions //
////////////////////////////

constexpr int N_LAG = 5; // Timesteps to skip to allow for response lag

constexpr double TIMESTEP = 0.020; // 20 ms
constexpr int N_STEPS = 100;
constexpr double HORIZON = TIMESTEP * N_STEPS;

constexpr int N_STEPS_LANE_CHECK = 20; // How often to check for lane change
constexpr int N_LANES = 3;

///////////////////////
// Speed definitions //
///////////////////////

constexpr double MPH_TO_METERS_PER_SECOND = 0.44704;

constexpr double MAX_MPH = 50;
constexpr double TARGET_MPH = 47;
constexpr double PASSING_MPH = 42;

constexpr double MAX_SPEED = MAX_MPH * MPH_TO_METERS_PER_SECOND;
constexpr double TARGET_SPEED = TARGET_MPH * MPH_TO_METERS_PER_SECOND;
constexpr double PASSING_SPEED = PASSING_MPH * MPH_TO_METERS_PER_SECOND;

constexpr double FOLLOW_DISTANCE = 20;
constexpr double LANE_WIDTH = 4;

///////////////////
// Costs for PTG //
///////////////////

constexpr int PTG_N_STEPS = 4;
constexpr double PTG_TIMESTEP = 0.040;
constexpr int PTG_N_SAMPLES = 10;

constexpr double COLLISION_COST          = 1e5;
constexpr double STAYS_ON_ROAD_COST      = 1e5;
constexpr double SPEED_LIMIT_COST        = 1e5;
constexpr double TOTAL_JERK_COST         = 1e3;
constexpr double TOTAL_ACCELERATION_COST = 1e3;
constexpr double D_DIFF_COST             = 10;
constexpr double S_DIFF_COST             = 10;
constexpr double BUFFER_COST             = 5;
constexpr double MAX_JERK_COST           = 1;
constexpr double MAX_ACCELERATION_COST   = 1;
constexpr double EFFICIENCY_COST         = 0;
constexpr double TIME_DIFF_COST          = 0;

constexpr double PTG_SIGMA_S = 20.;
constexpr double PTG_SIGMA_S_DOT = 4.;
constexpr double PTG_SIGMA_S_DDOT = 2.;
constexpr double PTG_SIGMA_D = 1.;
constexpr double PTG_SIGMA_D_DOT = 1.;
constexpr double PTG_SIGMA_D_DDOT = 1.;

constexpr double VEHICLE_RADIUS = 1.5;  // model vehicle as circle to simplify collision detection

constexpr double MAX_JERK = 10;// m/s/s/s
constexpr double MAX_ACCEL = 10;// m/s/s
constexpr double EXPECTED_JERK_IN_ONE_SEC = 2;// m/s/s
constexpr double EXPECTED_ACC_IN_ONE_SEC = 1; // m/s

///////////////////////////
// Behaviour definitions //
///////////////////////////

// Distance to check for car to perform a lane change
constexpr double RIGHT_LANE_CHECK_FRONT=25;
constexpr double RIGHT_LANE_CHECK_BACK=20;

constexpr double LEFT_LANE_CHECK_FRONT=25;
constexpr double LEFT_LANE_CHECK_BACK=20;

constexpr double PASSING_DISTANCE=30;

#endif //PATH_PLANNING_DEFINITIONS_H
