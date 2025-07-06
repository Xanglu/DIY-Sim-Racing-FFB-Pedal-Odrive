#pragma once

#include <stdint.h>
#include "Main.h"


/*  Uses ADS1256 */
class LoadCell_ADS1220 {
private:
  float _zeroPoint = 0.0;
  float _varianceEstimate = 0.0;
  float _standardDeviationEstimate = 0.0;

public:
  float getReadingKg() const;
  void setLoadcellRating(uint8_t loadcellRating_u8) const;
  void estimateBiasAndVariance();
};
