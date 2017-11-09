//
// Created by Cédric Bodet on 09.11.17.
//

#ifndef PATH_PLANNING_DEFINITIONS_H_H
#define PATH_PLANNING_DEFINITIONS_H_H

////////////////////////////
// Simulation definitions //
////////////////////////////

constexpr double TIMESTEP = .2; // s
constexpr int N_STEP = 50;

////////////////////////////////
// Physical world definitions //
////////////////////////////////

constexpr double MPH_TO_METERS_PER_SECOND = 0.44704;
constexpr double TARGET_MPH = 49.5;
constexpr double TARGET_SPEED = TARGET_MPH*MPH_TO_METERS_PER_SECOND;

//////////////////////////
// Control definitions //
/////////////////////////

constexpr double FORWARD_DISTANCE = 30;
constexpr double SPEED_STEP = 0.01;

#endif //PATH_PLANNING_DEFINITIONS_H_H
