//
// Created by Cédric Bodet on 05.11.17.
//

#include "Vehicle.h"

Vehicle::Vehicle(const State& startState) : start(startState) {}

const State Vehicle::stateAt(double t) const {
  // Implementation of MRUA in s and d direction
  // MRUA = Mouvement Rectiligne Uniformément Accéléré
  return State{
      .s=start.s+start.s_dot*t+.5*start.s_ddot*t*t,
      .s_dot=start.s_dot+start.s_ddot*t,
      .s_ddot=start.s_ddot,
      .d=start.d+start.d_dot*t+.5*start.d_ddot*t*t,
      .d_dot=start.d_dot+start.d_ddot*t,
      .d_ddot=start.d_ddot,
      .t=t
  };
}
