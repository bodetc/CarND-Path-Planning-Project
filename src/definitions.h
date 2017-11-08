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
constexpr int N_STEPS = 50;
constexpr double HORIZON = TIMESTEP*N_STEPS;

///////////////////////
// Speed definitions //
///////////////////////

constexpr double MPH_TO_METERS_PER_SECOND = 0.44704;

constexpr double MAX_MPH = 50;
constexpr double TARGET_MPH = 45;

constexpr double MAX_SPEED = MAX_MPH*MPH_TO_METERS_PER_SECOND;
constexpr double TARGET_SPEED = TARGET_MPH*MPH_TO_METERS_PER_SECOND;

///////////////////
// Costs for PTG //
///////////////////

constexpr int PTG_N_STEPS = 4;
constexpr double PTG_TIMESTEP = 0.040;
constexpr int PTG_N_SAMPLES = 10;

constexpr double TIME_DIFF_COST = 1;
constexpr double S_DIFF_COST = 10;
constexpr double D_DIFF_COST = 100;
constexpr double COLLISION_COST = 1000000;

constexpr double PTG_SIGMA_S = 10.;
constexpr double PTG_SIGMA_S_DOT = 4.;
constexpr double PTG_SIGMA_S_DDOT = 2.;
constexpr double PTG_SIGMA_D = 1.;
constexpr double PTG_SIGMA_D_DOT = 1.;
constexpr double PTG_SIGMA_D_DDOT = 1.;

constexpr double VEHICLE_RADIUS = 1.5;  // model vehicle as circle to simplify collision detection

#endif //PATH_PLANNING_DEFINITIONS_H
