#include "SignalFilter_2nd_order.h"





// v = s / t
// a = v/t
// a = s / t^2
// a = 300 / delta_t^2
// adjust model noise here s = 1/6 * j * delta_t^3 --> j = 6 * s / delta_t^3
//static const float KF_MODEL_NOISE_FORCE_ACCELERATION = ( 2.0f * 1000.0f / 0.05f/ 0.05f );
static const float KF_MODEL_NOISE_FORCE_JERK = 2000 * ( 2.0f * 4.0f / 0.1f/ 0.1f );

//static const float KF_MODEL_NOISE_FORCE_ACCELERATION = 180 * 1e6;//( 2.0f * 4.0f / 0.1f/ 0.1f );


KalmanFilter_2nd_order::KalmanFilter_2nd_order(float varianceEstimate)
  : _timeLastObservation(micros())
{
  // evolution matrix. Size is <Nstate,Nstate>
  _K.F = {1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};

  // command matrix.  Size is <Nstate,Ncom>
  _K.B = {1.0, 
          0.0,
          0.0 };
        
  // measurement matrix. Size is <Nobs,Nstate>
  _K.H = {1.0, 0.0, 0.0};

  // model covariance matrix. Size is <Nstate,Nstate>
  _K.Q = {1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};

  // measurement covariance matrix. Size is <Nobs,Nobs>
  _K.R = { varianceEstimate };
}

float KalmanFilter_2nd_order::filteredValue(float observation, float command, uint8_t modelNoiseScaling_u8) {
  // obtain time
  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - _timeLastObservation;
  float modelNoiseScaling_fl32 = modelNoiseScaling_u8;
  modelNoiseScaling_fl32 /= 255.0;

  if (modelNoiseScaling_fl32< 0.001)
  {
    modelNoiseScaling_fl32 = 0.001; 
  }
  if (elapsedTime < 1) { elapsedTime=1; }
  _timeLastObservation = currentTime;

  // update state transition and system covariance matrices
  float delta_t = (float)elapsedTime / 1000000.0f; // convert to seconds
  float delta_t_pow2 = delta_t * delta_t;
  float delta_t_pow3 = delta_t_pow2 * delta_t;
  float delta_t_pow4 = delta_t_pow2 * delta_t_pow2;

  _K.F = {1.0,  delta_t,  0.5f * delta_t * delta_t,
          0.0,  1.0, delta_t,
          0.0, 0.0, 1.0};

  _K.B = {1.0, 
          0.0,
          0.0};

  float Q11 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (1. / 6. * delta_t * delta_t * delta_t);
  float Q12 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (1. / 2. * delta_t * delta_t);
  float Q13 = KF_MODEL_NOISE_FORCE_JERK * (1. / 6. * delta_t * delta_t * delta_t) * (delta_t);

  float Q21 = Q12;
  float Q22 = KF_MODEL_NOISE_FORCE_JERK * (1. / 2. * delta_t * delta_t) * (1. / 2. * delta_t * delta_t);
  float Q23 = KF_MODEL_NOISE_FORCE_JERK * (1. / 2. * delta_t * delta_t) * (delta_t);

  float Q31 = Q13;
  float Q32 = Q23;
  float Q33 = KF_MODEL_NOISE_FORCE_JERK * (delta_t) * (delta_t);

  _K.Q = {  Q11, Q12, Q13,
            Q21, Q22, Q23,
            Q31, Q32, Q33};

        

  // APPLY KALMAN FILTER
  _K.update({observation}, {command});
  return _K.x(0,0);
}

float KalmanFilter_2nd_order::changeVelocity() {
  return _K.x(0,1);
}

float KalmanFilter_2nd_order::changeAccel() {
  return _K.x(0,2);
}
