#pragma once

#include "DiyActivePedal_types.h"
#include "StepperWithLimits.h"
#include "FastTrig.h"


static inline float sledPositionInMM(StepperWithLimits* stepper, DAP_config_st * config_st, float motorRevolutionsPerStep_fl32) {
  float currentPos = stepper->getCurrentPositionFromMin();
  return currentPos * motorRevolutionsPerStep_fl32 * (float)config_st->payLoadPedalConfig_.spindlePitch_mmPerRev_u8;
}

static inline float sledPositionInMM_withPositionAsArgument(float currentPos_fl32, DAP_config_st * config_st, float motorRevolutionsPerStep_fl32) {
  return currentPos_fl32 * motorRevolutionsPerStep_fl32 * (float)config_st->payLoadPedalConfig_.spindlePitch_mmPerRev_u8;
}

static inline float pedalInclineAngleDeg(float sledPositionMM, DAP_config_st * config_st) {
  // see https://de.wikipedia.org/wiki/Kosinussatz
  // A: is lower pedal pivot
  // C: is upper pedal pivot
  // B: is rear pedal pivot
  float a = (float)config_st->payLoadPedalConfig_.lengthPedal_a;
  float b = (float)config_st->payLoadPedalConfig_.lengthPedal_b;
  float c_ver = (float)config_st->payLoadPedalConfig_.lengthPedal_c_vertical;
  float c_hor = (float)config_st->payLoadPedalConfig_.lengthPedal_c_horizontal + sledPositionMM;
  float c = sqrtf(c_ver * c_ver + c_hor * c_hor);
  

//#define DEBUG_PEDAL_INCLINE
#ifdef DEBUG_PEDAL_INCLINE
  Serial.print("a: ");    Serial.print(a);
  Serial.print(", b: ");  Serial.print(b);
  Serial.print(", c: ");  Serial.print(c);

  Serial.print(", sledPositionMM: ");  Serial.print(sledPositionMM);
#endif

  float nom = b*b + c*c - a*a;
  float den = 2.0f * b * c;
  
  float alpha = 0.0f;
  if (abs(den) > 0.01f) {
    // alpha = acos( nom / den );
    // alpha = fastAcos( nom / den );
    alpha = iacos( nom / den ) * DEG_TO_RAD;
  }

#ifdef DEBUG_PEDAL_INCLINE
  Serial.print(", alpha1: ");  Serial.print(alpha * RAD_TO_DEG);
#endif

  // add incline due to AB incline --> result is incline realtive to horizontal 
  if (abs(c_hor)>0.01f) {
    // alpha += atan2f(c_ver, c_hor); // y, x
    alpha += atan2Fast(c_ver, c_hor); // y, x
  }





#ifdef DEBUG_PEDAL_INCLINE
  Serial.print(", alpha2: ");  Serial.print(alpha * RAD_TO_DEG);
  Serial.println(" ");
#endif

  
  return alpha * RAD_TO_DEG;
}


static inline float convertToPedalForce(float F_l, float sledPositionMM, DAP_config_st * config_st) {
  // see https://de.wikipedia.org/wiki/Kosinussatz
  // A: is lower pedal pivot
  // B: is rear pedal pivot
  // C: is upper pedal pivot
  // D: is foot rest
  //
  // a: is loadcell rod (connection CB)
  // b: is lower pedal plate (connection AC)
  // c: is sled line (connection AC)
  // d: is upper pedal plate  (connection AC)

  float a = (float)config_st->payLoadPedalConfig_.lengthPedal_a;
  float b = (float)config_st->payLoadPedalConfig_.lengthPedal_b;
  float d = (float)config_st->payLoadPedalConfig_.lengthPedal_d;

  float c_ver = (float)config_st->payLoadPedalConfig_.lengthPedal_c_vertical;
  float c_hor = (float)config_st->payLoadPedalConfig_.lengthPedal_c_horizontal + sledPositionMM;
  float c = sqrtf(c_ver * c_ver + c_hor * c_hor);
  


  //Serial.print("a: ");    Serial.print(a);
  //Serial.print(", b: ");  Serial.print(b);
  //Serial.print(", c: ");  Serial.print(c);
  //Serial.print(", d: ");  Serial.print(d);
  //Serial.print(", sled: ");  Serial.print(sledPositionMM);
  //Serial.print(", b_hor: ");  Serial.print(b_hor);
  //Serial.println();


  // lower plus upper pedal plate length
  float b_plus_d = fabsf(b + d);

  // compute gamma angle, see https://de.wikipedia.org/wiki/Kosinussatz
  float nom = a*a + b*b - c*c;
  float den = 2 * a * b;
  
  float arg = 0.0f;
  if (abs(den) > 0.01f) {
    arg = nom / den;
    arg *= arg;
  }

  // apply conversion factor to loadcell reading 
  float one_minus_arg = 1.0f - arg;
  float F_b  = F_l;
  if ( (b_plus_d > 0.0f) && (one_minus_arg > 0.0f) )
  {
     F_b *= b / (b_plus_d) * sqrtf( one_minus_arg );
  }
  
  
  return F_b;
}


// Calculate gradient of phi with respect to sled position.
// This is done by taking the derivative of the force with respect to the sled position.
static inline float convertToPedalForceGain(float sledPositionMM, DAP_config_st * config_st) {
  // see https://de.wikipedia.org/wiki/Kosinussatz
  // A: is lower pedal pivot
  // B: is rear pedal pivot
  // C: is upper pedal pivot
  // D: is foot rest
  //
  // a: is loadcell rod (connection CB)

  // b: is lower pedal plate (connection AC)
  // c: is sled line (connection AC)
  // d: is upper pedal plate  (connection AC)

  float a = (float)config_st->payLoadPedalConfig_.lengthPedal_a;
  float b = (float)config_st->payLoadPedalConfig_.lengthPedal_b;
  float d = (float)config_st->payLoadPedalConfig_.lengthPedal_d;

  float c_ver = (float)config_st->payLoadPedalConfig_.lengthPedal_c_vertical;
  float c_hor = (float)config_st->payLoadPedalConfig_.lengthPedal_c_horizontal + sledPositionMM;
  float c = sqrtf(c_ver * c_ver + c_hor * c_hor);
  

  // float alpha = acos( (b*b + c*c - a*a) / (2.0f*b*c) );
  // float alpha = fastAcos( (b*b + c*c - a*a) / (2.0f*b*c) );
  float alpha = iacos( (b*b + c*c - a*a) / (2.0f*b*c) ) * DEG_TO_RAD;


  // float alphaPlus = atan2f(c_ver, c_hor); // y, x
  float alphaPlus = atan2Fast(c_ver, c_hor); // y, x

  

  // float sinAlpha = sin(alpha);
  // float cosAlpha = cos(alpha);
  // float sinAlphaPlus = sin(alphaPlus);
  // float cosAlphaPlus = cos(alphaPlus);

  float alphaInDeg = alpha * RAD_TO_DEG;
  float alphaPlusInDeg = alphaPlus * RAD_TO_DEG;
  float sinAlpha = isin(alphaInDeg);
  float cosAlpha = icos(alphaInDeg);
  float sinAlphaPlus = isin(alphaPlusInDeg);
  float cosAlphaPlus = icos(alphaPlusInDeg);

  // d_alpha_d_x
  float d_alpha_d_x = - 1.0f / fabs( sinAlpha ) * ( 1.0f / b - cosAlpha / c) * cosAlphaPlus;

  // d_alphaPlus_d_x
  float d_alphaPlus_d_x = - c_ver / (c * c);

  float d_phi_d_x = d_alpha_d_x + d_alphaPlus_d_x;

  // return in deg/mm
  return d_phi_d_x * RAD_TO_DEG;
}