//
// Created by Cédric Bodet on 28.10.17.
//

#include <profile.h>

#ifndef PATH_PLANNING_DEFINITIONS_H
#define PATH_PLANNING_DEFINITIONS_H

////////////////////////////
// Simulation definitions //
////////////////////////////

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

#endif //PATH_PLANNING_DEFINITIONS_H
