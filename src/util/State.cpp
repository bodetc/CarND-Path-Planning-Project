//
// Created by Cédric Bodet on 09.11.17.
//

#include "State.h"

const State State::stateAt(double T) const {
  const double dt = (T-t);
  // Implementation of MRUA in s and d direction
  // MRUA = Mouvement Rectiligne Uniformément Accéléré
  return State{
      .s=s+s_dot*dt+.5*s_ddot*dt*dt,
      .s_dot=s_dot+s_ddot*dt,
      .s_ddot=s_ddot,
      .d=d+d_dot*dt+.5*d_ddot*dt*dt,
      .d_dot=d_dot+d_ddot*dt,
      .d_ddot=d_ddot,
      .t=t
  };
}