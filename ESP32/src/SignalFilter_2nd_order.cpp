#include "SignalFilter_2nd_order.h"

// Define constants from the original code
static const float KF_MODEL_NOISE_FORCE_JERK = 8.0f * 1e9;
// Constructor
KalmanFilter_2nd_order::KalmanFilter_2nd_order(float varianceEstimate)
  : _timeLastObservation(micros()), _R(varianceEstimate)
{
  // Initialize state
  _x[0] = _x[1] = _x[2] = 0.0f;

  // Initialize error covariance
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          _P_cov[i][j] = 0.0f;
      }
  }

  // Measurement matrix (position measurement only)
  _H[0][0] = 1.0f;
  _H[0][1] = 0.0f;
  _H[0][2] = 0.0f;
}

// Matrix multiplication for 3x3 matrices
void KalmanFilter_2nd_order::multiplyMatrices(float mat1[3][3], float mat2[3][3], float result[3][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      result[i][j] = 0.0f;
      for (int k = 0; k < 3; ++k)
        result[i][j] += mat1[i][k] * mat2[k][j];
    }
}

// Matrix-vector multiplication for 3x3 and 3x1
void KalmanFilter_2nd_order::multiplyMatrices_3x3_3x1(float mat1[3][3], float vec1[], float result[]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0.0f;
        for (int k = 0; k < 3; ++k) {
            result[i] += mat1[i][k] * vec1[k];
        }
    }
}

float KalmanFilter_2nd_order::filteredValue(float observation, float command, uint8_t modelNoiseScaling_u8) {
  // Obtain time
  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - _timeLastObservation;
  _timeLastObservation = currentTime;

  float modelNoiseScaling = modelNoiseScaling_u8 / 255.0f;
  if (modelNoiseScaling < 0.001f) modelNoiseScaling = 0.001f;
  if (elapsedTime < 1) elapsedTime = 1;
  if (elapsedTime > 5000) elapsedTime = 5000;

  float delta_t = elapsedTime / 1000000.0f;
  float delta_t_pow2 = delta_t * delta_t;
  float delta_t_pow3 = delta_t_pow2 * delta_t;
  float delta_t_pow4 = delta_t_pow2 * delta_t_pow2;
  float delta_t_pow5 = delta_t_pow4 * delta_t;
  float delta_t_pow6 = delta_t_pow5 * delta_t;

  // Update F matrix based on time step
  // F = [1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1]
  _F[0][0] = 1.0f; _F[0][1] = delta_t; _F[0][2] = 0.5f * delta_t_pow2;
  _F[1][0] = 0.0f; _F[1][1] = 1.0f; _F[1][2] = delta_t;
  _F[2][0] = 0.0f; _F[2][1] = 0.0f; _F[2][2] = 1.0f;

  // Update Q matrix based on time step and jerk  
  // jerk to [position; velocity; acceleration] --> r = [1/6 * delta_t^3; 1/2 * delta_t^2; delta_t]
  // Q = r * a_var_jerk * r'
  //   = r * r' * a_var_jerk
  //   = [1/36*delta_t^6, 1/12*delta_t^5, 1/6*delta_t^4;
  //   =  1/12*delta_t^5, 1/4*delta_t^4, 1/2*delta_t^3;
  //   =  1/6*delta_t^4, 1/2*delta_t^3, delta_t^2]
  float a_var_jerk = modelNoiseScaling * KF_MODEL_NOISE_FORCE_JERK;
  _Q[0][0] = a_var_jerk * delta_t_pow6 / 36.0f;
  _Q[0][1] = a_var_jerk * delta_t_pow5 / 12.0f;
  _Q[0][2] = a_var_jerk * delta_t_pow4 / 6.0f;
  _Q[1][0] = _Q[0][1];
  _Q[1][1] = a_var_jerk * delta_t_pow4 / 4.0f;
  _Q[1][2] = a_var_jerk * delta_t_pow3 / 2.0f;
  _Q[2][0] = _Q[0][2];
  _Q[2][1] = _Q[1][2];
  _Q[2][2] = a_var_jerk * delta_t_pow2;

  // Predict Step
  // x_pred = F * x
  float x_pred[3];
  multiplyMatrices_3x3_3x1(_F, _x, x_pred);
  
  // P_pred = F * P * F' + Q
  float Ftrans[3][3] = {
    { _F[0][0], _F[1][0], _F[2][0] },
    { _F[0][1], _F[1][1], _F[2][1] },
    { _F[0][2], _F[1][2], _F[2][2] }
  };
  float FP[3][3];
  float FPFtrans[3][3];
  multiplyMatrices(_F, _P_cov, FP);
  multiplyMatrices(FP, Ftrans, FPFtrans);
  float P_pred[3][3];
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          P_pred[i][j] = FPFtrans[i][j] + _Q[i][j];
      }
  }
  
  // Update Step
  // y = z - H * x_pred
  // Since observation is position (1x1), we don't need to include the velocity and acceleration prediction
  float y = observation - ( _H[0][0] * x_pred[0] );
  
  // S = H * P_pred * H' + R
  // H = [1; 0; 0]
  // P = 3x3
  // H * P = [P_00, P_01, P_02; 0, 0, 0; 0, 0, 0]
  // [P_00, P_01, P_02; 0, 0, 0; 0, 0, 0] * [1, 0, 0] = [P_00, 0, 0; 0, 0, 0; 0, 0, 0]
  // H * P_pred * H' simplifies to P_pred[0][0]
  
  // S_cov = [P_00, 0, 0; 0, 0, 0; 0, 0, 0] + R
  float S_cov = P_pred[0][0] + _R;
  
  
  // inv(S_cov) = 1 / (P_00 + R)
  float inv_S_cov = 1.0f / S_cov;
  if (fabsf(S_cov) > 0.000001f) {
    // K = P_pred * H' * inv(S)
	
	// P_pred * H' = 3x3 * [1, 0, 0]
	// K = 3x3 but only first column is not zero
    _K[0] = P_pred[0][0] * inv_S_cov;
    _K[1] = P_pred[1][0] * inv_S_cov;
    _K[2] = P_pred[2][0] * inv_S_cov;

    // Update state estimate: x = x_pred + K * y
    _x[0] = x_pred[0] + _K[0] * y;
    _x[1] = x_pred[1] + _K[1] * y;
    _x[2] = x_pred[2] + _K[2] * y;

    // Update error covariance: P = (I - K*H) * P_pred
	// K = 3x3 
	// H = 3x1 with only first element beeing non zero
	// --> K*H = 3x3 = [_K[0]*_H[0], 0, 0; 0, 0, 0; 0, 0, 0]
	float KH[3][3];
    //for (int i = 0; i < 3; ++i) {
    //  for (int j = 0; j < 3; ++j) {
    //    KH[i][j] = _K[i] * _H[0][j];
    //  }
    //}
	KH[0][0] = _K[0] * _H[0][0];
	KH[0][1] = 0.0f;
	KH[0][2] = 0.0f;
	KH[1][0] = 0.0f;
	KH[1][1] = 0.0f;
	KH[1][2] = 0.0f;
	KH[1][0] = 0.0f;
	KH[1][1] = 0.0f;
	KH[1][2] = 0.0f;
	
    float I_KH[3][3];
    //for (int i = 0; i < 3; ++i) {
    //  for (int j = 0; j < 3; ++j) {
    //    if (i == j) I_KH[i][j] = 1.0f - KH[i][j];
    //    else I_KH[i][j] = -KH[i][j];
    //  }
    //}
	
	I_KH[0][0] = 1.0f - KH[0][0];
	I_KH[0][1] = 0.0f;
	I_KH[0][2] = 0.0f;
	I_KH[1][0] = 0.0f;
	I_KH[1][1] = 1.0f;
	I_KH[1][2] = 0.0f;
	I_KH[1][0] = 0.0f;
	I_KH[1][1] = 0.0f;
	I_KH[1][2] = 1.0f;
	
	
    multiplyMatrices(I_KH, P_pred, _P_cov);

  } else {
    // S is zero, reset P_cov to avoid issues
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        _P_cov[i][j] = 0.0f;
      }
    }
  }

  return _x[0];
}

float KalmanFilter_2nd_order::changeVelocity() {
  return _x[1];
}

float KalmanFilter_2nd_order::changeAccel() {
  return _x[2];
}
