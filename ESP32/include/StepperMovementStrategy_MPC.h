#pragma once
#include "DiyActivePedal_types.h"
#include "Main.h"
#include "StepperWithLimits.h"
#include "ForceCurve.h"

unsigned long lastTime = 0;
float output = 0.0f;       // PID-Ausgang
float filteredOutput = 0.0f;
bool expFilterHasBeenInitialized = false;
// Filter-Variablen
float RC = 1.0f / (2.0f * 3.14159f * 100.0f); // RC für 20 Hz Eckfrequenz
float MPC_0_gain=8.2f;
int printStep = 0;

float posArray[10] = {0.0f};
uint8_t arrayIndex = 0;
int32_t MoveByForceTargetingStrategy(float loadCellReadingKg, StepperWithLimits* stepper, ForceCurve_Interpolated* forceCurve, const DAP_calculationVariables_st* calc_st, DAP_config_st* config_st, float absForceOffset_fl32, float changeVelocity, float d_phi_d_x, float d_x_hor_d_phi) {
  // see https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal/wiki/Movement-control-strategies#mpc


  /*
  This closed-loop control strategy models the foot as a spring with a certain stiffness k1.
  The force resulting from that model is F1 = k1 * x. 
  To find the servo target position:
  1) A line with the slope -k1 at the point of the loadcell reading & current position is drawn.
  2) The intersection with the force-travel curve gives the target position
  
  Since the force-travel curve might be non-linear, Newtons method is used to numerically find the intersection point.
  f(x_n) = -k1 * x + b - forceCurve(x) = 0
  x_n+1 = x_n - f(x_n) / f'(x_n)
  whereas x_n is the servo position at iteration n
  f(x_n) = -k1 * x + b - forceCurve(x)
  f'(x_n) = -k1 - d(forceCurve(x)) / dx
  */

  
  
  // get current stepper position
  float stepperPos = stepper->getCurrentPositionFromMin();

  float estimatedServoPosErrorInSteps_fl32 = stepper->getEstimatedPosError();
  if (0)
  {
    stepperPos += estimatedServoPosErrorInSteps_fl32;
  }


  // float lagedPos;

  // posArray[arrayIndex] = stepperPos;
  // arrayIndex++;
  // arrayIndex %= 10;

  // lagedPos = 



  // for (uint8_t aryIdx = 1; aryIdx <= 9; aryIdx++)
  // {
  //   posArray[aryIdx-1] = posArray[aryIdx];
  // }
  // posArray[9] = stepperPos;

  // uint8_t lag = 9-2;
  // stepperPos = posArray[lag];

  
  // stepperPos = posArray[lag];
  

  // // get iSVs position
  // float stepperPos2 = stepper->getServosInternalPositionCorrected() - stepper->getMinPosition();
  // stepperPos = stepperPos2;

  // ActiveSerial->printf("ESP pos: %f,    iSV pos: %f\n", stepperPos, stepperPos2);
  // delay(20);

  if (false == expFilterHasBeenInitialized)
  {
    filteredOutput = stepperPos;
    expFilterHasBeenInitialized = true;
  }

  // get current stepper velocity
  int32_t currentSpeedInMilliHz = stepper->getCurrentSpeedInMilliHz();
  uint32_t maxSpeedInMilliHz = stepper->getMaxSpeedInMilliHz();
  float speedNormalized_fl32 = ( (float)currentSpeedInMilliHz ) / ((float)maxSpeedInMilliHz)  ; // 250000000 --> 250
  float speedAbsNormalized_fl32 = constrain( fabsf(speedNormalized_fl32), 0.1f, 1.0f);
  float oneMinusSpeedNormalized_fl32 = 1.0f - speedAbsNormalized_fl32;
  
  // motion corrected loadcell reading
  float loadCellReadingKg_corrected = loadCellReadingKg;

  // set initial guess
  float stepperPos_initial = stepperPos;

  // foot spring stiffness
  float d_f_d_x_hor = -1.0f*MPC_0_gain;

  // velocity dependent foot spring stiffness 
  float d_f_t_d_x_hor = 0.0f;

  // acceleration dependent foot spring stiffness 
  float d_f_tt_d_x_hor = 0.0f;


  // ActiveSerial->printf("MPC 0: %f,    1:%f,    2:%f\n", config_st->payLoadPedalConfig_.MPC_0th_order_gain, config_st->payLoadPedalConfig_.MPC_1st_order_gain, config_st->payLoadPedalConfig_.MPC_2nd_order_gain);
  // delay(20);

  // how many mm movement to order if 1kg of error force is detected
  // this can be tuned for responsiveness vs oscillation
  float mm_per_motor_rev = config_st->payLoadPedalConfig_.spindlePitch_mmPerRev_u8;//TRAVEL_PER_ROTATION_IN_MM;
  float steps_per_motor_rev = (float)calc_st->stepsPerMotorRevolution; //(float)STEPS_PER_MOTOR_REVOLUTION;

  float mmPerStep = 0.0f;
  if (steps_per_motor_rev > 0.0f)
  {
    mmPerStep = mm_per_motor_rev / steps_per_motor_rev ;
  }

  // compute d(x_hor) / d(step) from chain rule
  // d(x_hor) / d(step) = ( d(x_hor) / d(phi) ) * [ d(phi)/d(x) ] * { d(x)/d(step) }
  float d_x_hor_d_step = (-d_x_hor_d_phi) * (-d_phi_d_x) * mmPerStep;

  // ActiveSerial->printf("PosFraction: %f,    pos:%f,    travelRange:%f,    posMin:%d,    posMax:%d\n", stepper->getCurrentPositionFractionFromExternalPos( stepperPos ), stepperPos, stepper->getCurrentTravelRange(),  calc_st->stepperPosMin, calc_st->stepperPosMax );
  // delay(20);

  // ActiveSerial->printf("speed: %f,    maxSpeed:%f\n", (float)currentSpeedInMilliHz, (float)maxSpeedInMilliHz);
  // delay(20);

  // velocity dependent force in kg = (kg*s/step) * (step/s)
  float forceInKgAndSecondPerStep_fl32 = d_f_t_d_x_hor * d_x_hor_d_step;
  float velocityDependingForceInKg_fl32 = forceInKgAndSecondPerStep_fl32 * (currentSpeedInMilliHz / 1000.0f);

  // acceleration dependent force in kg = (kg*s^2/step) * (step/(s^2))
  float forceInKgAndSecondSquarePerStep_fl32 = d_f_tt_d_x_hor * d_x_hor_d_step;
  // Todo: compute acceleration dependet force
  

  // correct loadcell reading with velocity and acceleration readings
  float expectedCycleTime = 0.001f;
  // loadCellReadingKg_corrected -= velocityDependingForceInKg_fl32;

  // [mmPerStep] = mm/step, e.g. 0.001563 = 10mm/rev / 6400steps/rev
  // [d_phi_d_x] = deg/mm e.g. -0.305367
  // [d_x_hor_d_phi] = mm/deg, e.g. -3.832119
  // [d_x_hor_d_step] = mm/step, e.g. 0.001458
  // if (printStep > 10)
  // {
  //   ActiveSerial->printf("Vel:%f,    mmPerStep:%f,    d_phi_d_x:%f,    d_x_hor_d_phi:%f,    d_x_hor_d_step:%f,    force:%f\n", speedNormalized_fl32, mmPerStep, d_phi_d_x, d_x_hor_d_phi, d_x_hor_d_step, velocityDependingForceInKg_fl32);
  //   printStep = 0;
  // }
  // else{
  //   printStep++;
  // }
  

 
  // Find the intersections of the force curve and the foot model via Newtons-method
  #define MAX_NUMBER_OF_NEWTON_STEPS 5
  // int64_t stepUpdates[MAX_NUMBER_OF_NEWTON_STEPS];
  for (uint8_t iterationIdx = 0; iterationIdx < MAX_NUMBER_OF_NEWTON_STEPS; iterationIdx++)
  {
    //float stepperPosFraction = stepper->getCurrentPositionFraction();
    float stepperPosFraction = stepper->getCurrentPositionFractionFromExternalPos( stepperPos );
  
    // clamp the stepper position to prevent problems with the spline
    float x_0 = constrain(stepperPosFraction, 0.0f, 1.0f);
    
    // get force and force gradient of force vs travel curve
    float loadCellTargetKg = forceCurve->EvalForceCubicSpline(config_st, calc_st, x_0);
    float gradient_force_curve_fl32 = forceCurve->EvalForceGradientCubicSpline(config_st, calc_st, x_0, false);

    // Convert loadcell reading to pedal force
    // float sledPosition = sledPositionInMM_withPositionAsArgument(x_0 * stepper->getTravelSteps(), config_st, motorRevolutionsPerSteps_fl32);
    // float pedalInclineAngleInDeg_fl32 = pedalInclineAngleDeg(sledPosition, config_st);
    // // float pedalForce_fl32 = convertToPedalForce(loadcellReading, sledPosition, &dap_config_pedalUpdateTask_st);
    // float d_phi_d_x_2 = convertToPedalForceGain(sledPosition, config_st);

    // // compute gain for horizontal foot model
    // float b = config_st->payLoadPedalConfig_.lengthPedal_b;
    // float d = config_st->payLoadPedalConfig_.lengthPedal_d;
    // float d_x_hor_d_phi_2 = -(b+d) * sinf(pedalInclineAngleInDeg_fl32 * DEG_TO_RAD_FL32);

    // apply effect force offset
    // loadCellTargetKg -= absForceOffset_fl32;

    // make stiffness dependent on force curve gradient
    // less steps per kg --> steeper line
    float gradient_normalized_force_curve_fl32 = forceCurve->EvalForceGradientCubicSpline(config_st, calc_st, x_0, true);
    gradient_normalized_force_curve_fl32 = constrain(gradient_normalized_force_curve_fl32, 0.05f, 1.0f);

    // compute force error
    float forceError_fl32 = loadCellReadingKg_corrected - loadCellTargetKg;

    // angular foot model
    // m1 = d_f_d_x dForce / dx
    //float m1 = d_f_d_phi * (-d_phi_d_x);
    
    // Translational foot model
    // given in kg/step
    float m1 = d_f_d_x_hor * d_x_hor_d_step;

    // m1 *= oneMinusSpeedNormalized_fl32;
    
    // gradient of the force curve
    // given in kg/step
    float m2 = gradient_force_curve_fl32; 
    
    // ActiveSerial->printf("m1:%f,    m2:%f,    speed:%f\n", m1, m2, (float)currentSpeedInHz);
    // delay(20);

    // Newton update
    // float denom = m1 - m2 + d_f_t_d_x_hor * fabsf(m1) / speedAbsNormalized_fl32;
    // float denom = (m1 - m2) * (1.0f - config_st->payLoadPedalConfig_.MPC_1st_order_gain * fabsf(m1) / speedAbsNormalized_fl32 );
    float denom = m1 - m2;// - velocityDependingForceInKg_fl32;//config_st->payLoadPedalConfig_.MPC_1st_order_gain * oneMinusSpeedNormalized_fl32;
    
    if ( fabsf(denom) > 0.0f )
    {
      // https://en.wikipedia.org/wiki/Newton%27s_method
      // Newton algorithm
      // x(n+1) = x(n) + stepUpdate
      // x(n+1) = x(n) - f(x_n) / f'(x_n)
      float stepUpdate = - forceError_fl32 / ( denom );
      // a positive stepUpdate means sled moves away from the pedal.

      // smoothen update with force curve gradient since it had better results w/ clutch pedal characteristic
      // stepUpdate *= gradient_normalized_force_curve_fl32;
      // stepUpdate *= speedAbsNormalized_fl32;

      // update expected force reading
      // Todo: update expected force after step execution
      loadCellReadingKg_corrected += m1 * stepUpdate;

      // update position
      stepperPos += stepUpdate;

      // stop iteration
      if (fabsf(stepUpdate) < 2.0f)
      {
        break;
      }
      // stepUpdates[iterationIdx] = stepUpdate;
    }
  }

  // unsigned long now = micros();
  // float deltaTime = ( (float)(now - lastTime) ) / 1000000.0f; // in Sekunden
  // lastTime = now;

  

  // // Filter-Koeffizient berechnen
  // float alpha = deltaTime / (deltaTime + RC);

  // output = stepperPos;// - stepperPos_initial;

  // // Tiefpassfilter anwenden
  // filteredOutput = alpha * output + (1.0 - alpha) * filteredOutput;

  // stepperPos = filteredOutput;

  // ActiveSerial->printf("0:%i, 1:%i, 2:%i, 3:%i, 4:%i\n", stepUpdates[0], stepUpdates[1], stepUpdates[2], stepUpdates[3], stepUpdates[4]);
  // delay(20);
  
  // read the min position
  stepperPos += stepper->getMinPosition();

  // clamp target position to range
  int32_t posStepperNew = constrain(stepperPos, calc_st->stepperPosMin, calc_st->stepperPosMax );

  return posStepperNew;
}