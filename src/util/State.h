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
  double s, s_dot, s_ddot, d, d_dot, d_ddot, t;

  const State operator+(const State& other) const {
    return State {
        .s = s+other.s,
        .s_dot = s_dot+other.s_dot,
        .s_ddot = s_ddot+other.s_ddot,
        .d = d+other.d,
        .d_dot = d_dot+other.d_dot,
        .d_ddot = d_ddot+other.d_ddot,
        .t = t + other.t
    };
  }
};


#endif //PATH_PLANNING_STATE_H
