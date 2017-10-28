//
// Created by Cédric Bodet on 28.10.17.
//

#ifndef PATH_PLANNING_POLYNOMIALSOLVER_H
#define PATH_PLANNING_POLYNOMIALSOLVER_H

#include <vector>
#include "Polynomial.h"
#include "../util/Trajectory.h"

class PolynomialSolver {
public:

  /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
    INPUTS
      start - the vehicles start location given as a length three array
          corresponding to initial values of [s, s_dot, s_double_dot]
      end   - the desired end state for vehicle. Like "start" this is a
          length three array.
      T     - The duration, in seconds, over which this maneuver should occur.
    OUTPUT
      an array of length 6, each value corresponding to a coefficent in the polynomial
      s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    EXAMPLE
      > JMT( [0, 10, 0], [10, 10, 0], 1)
      [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
  static Polynomial solveJMT(std::vector<double> start, std::vector<double> end, double T);

//  static Trajectory solveJMT(std::vector<double> start_s, std::vector<double> end_s,
//                             std::vector<double> start_d, std::vector<double> end_d,
//                             double T, int n);
};


#endif //PATH_PLANNING_POLYNOMIALSOLVER_H
