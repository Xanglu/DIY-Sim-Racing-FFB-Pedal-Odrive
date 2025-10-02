#pragma once

#include <Kalman.h>

static const int Nobs_2nd_order = 1;      // 1 filter input:   observed value
static const int Nstate_2nd_order = 3;    // 2 filter outputs: change, velocity & acceleration
static const int Ncom_2nd_order = 1; // Number of commands, u vector


class KalmanFilter_2nd_order {
private:
  KALMAN<Nstate_2nd_order, Nobs_2nd_order, Ncom_2nd_order> _K;
  unsigned long _timeLastObservation;

public:
  KalmanFilter_2nd_order(float varianceEstimate);

  float filteredValue(float observation, float command, uint8_t modelNoiseScaling_u8);
  float changeVelocity();
  float changeAccel();
};
