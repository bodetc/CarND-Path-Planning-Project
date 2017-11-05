//
// Created by Cédric Bodet on 05.11.17.
//

#ifndef PATH_PLANNING_STATE_H
#define PATH_PLANNING_STATE_H


/**
 * Structure for storing vehicle states
 */
struct State {
public:
  double s, s_dot, s_ddot, d, d_dot, d_ddot;
};


#endif //PATH_PLANNING_STATE_H
