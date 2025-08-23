#include "LoadCell_ads1220.h"
#include "Main.h"
#include "Arduino.h"

#ifdef USES_ADS1220

#include <SPI.h>
#include <ADS1220_WE.h>


static const int NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION = 1000;
static const float DEFAULT_VARIANCE_ESTIMATE = 0.2f * 0.2f;
static const float LOADCELL_VARIANCE_MIN = 7.0 * 1e-5; // on 8th april 2025, approx. 50g fluctuation were observed --> 6 * sigma = 50g --> sigma = 50g / 6 = 8g --> sigma^2 = (8g)^2 = 0.00064

float updatedConversionFactor_f64 = 1.0f;
#define CONVERSION_FACTOR LOADCELL_WEIGHT_RATING_KG / (LOADCELL_EXCITATION_V * (LOADCELL_SENSITIVITY_MV_V/1000.0f))
#define TIMEOUT_FOR_DRDY_TO_BECOME_LOW (uint32_t)2000
#define DELAY_IN_US_FOR_DRDY_TO_BECOME_LOW (uint32_t)20

// reference voltage in milli-volts
float refVoltageInMV_fl32 = 5000.0f;

static SemaphoreHandle_t timer_fireLoadcellReadingReady_global;

// This is our Interrupt Service Routine
void IRAM_ATTR drdyInterrupt() {

  if(timer_fireLoadcellReadingReady_global != NULL)
  {
    // It immediately gives the semaphore to wake up myCore1Task.
    xSemaphoreGiveFromISR(timer_fireLoadcellReadingReady_global, NULL);
  }
}

/* Provides a singleton instance of the ADS1220 ADC driver. */
ADS1220_WE& getADC() {
  
  static SPIClass adsSPI(FSPI);  // Or use VSPI or HSPI for ESP32
  static ADS1220_WE adc(&adsSPI, FFB_ADS1220_CS, FFB_ADS1220_DRDY, true);
  
  //static ADS1220_WE adc(FFB_ADS1220_CS, FFB_ADS1220_DRDY);

  static bool firstTime = true;
  if (firstTime) {
    Serial.println("Initializing ADS1220 ADC...");

    // Initialize custom SPI bus. This should be done only once.
    adsSPI.begin(FFB_ADS1220_SCLK, FFB_ADS1220_DOUT, FFB_ADS1220_DIN, FFB_ADS1220_CS);

    // Initialize ADS1220
    if (!adc.init()) {
      Serial.println("ADS1220 not found!");
      while (1);
    }

    // ADS1220 Configuration
    adc.setDataRate(ADS1220_DR_LVL_6);     // 2000SPS

    // PGA
    adc.setGain(ADS1220_GAIN_128);            // Gain for load cell

    // reference voltage
    adc.setVRefSource(ADS1220_VREF_AVDD_AVSS);
    //ads.setVRefValue_V(4.7f);    // set reference voltage in volts
    adc.setAvddAvssAsVrefAndCalibrate();

    float refVolt_fl32 = adc.getVRef_V();
    refVoltageInMV_fl32 = refVolt_fl32 * 1000.0f; // convert to mV
    Serial.print("Reference voltage: ");
    Serial.println(refVolt_fl32);

    // differential channels
    adc.setCompareChannels(ADS1220_MUX_0_1);              // Differential AIN0 - AIN1

    // set modulalar frequency
    adc.setOperatingMode(ADS1220_TURBO_MODE);

    // continous reading mode
    adc.setConversionMode(ADS1220_CONTINUOUS);  // Add this line in setup

    // set 50HZ and 60Hz FIR filter
    adc.setFIRFilter(ADS1220_50HZ_60HZ);

    // set 
    //adc.setDrdyMode(ADS1220_DOUT_DRDY);
    adc.setDrdyMode(ADS1220_DRDY);

    // needs to wait fir DRDY come from low to high --> do not use
    adc.setNonBlockingMode(true); // switch ton non-blocking mode
    
    // assign interrupt to DRDY falling edge to make waiting more efficient
    attachInterrupt(digitalPinToInterrupt(FFB_ADS1220_DRDY), drdyInterrupt, FALLING);


    Serial.println("ADC Started");
    
    firstTime = false;
  }

  return adc;
}






LoadCell_ADS1220::LoadCell_ADS1220()
  : _zeroPoint(0.0f), _varianceEstimate(DEFAULT_VARIANCE_ESTIMATE)
{
  // differential channels
  getADC().setCompareChannels(ADS1220_MUX_0_1);              // Differential AIN0 - AIN1
  timer_fireLoadcellReadingReady_global = xSemaphoreCreateBinary();
}



void LoadCell_ADS1220::setLoadcellRating(uint8_t loadcellRating_u8) const {
  getADC(); // Ensure ADC is initialized
  
  updatedConversionFactor_f64 = 1.0f;
  if (LOADCELL_WEIGHT_RATING_KG>0)
  {
      float excitationVoltage = refVoltageInMV_fl32 / 1000.0f;
      float fullScale_mV = LOADCELL_SENSITIVITY_MV_V * excitationVoltage; // 2 mV/V * Vexc
      float loadcellRatingInGram_fl32 = (((float)loadcellRating_u8) * 1000.0f); // convert kg to gram
      float gramsPerMillivolt =  loadcellRatingInGram_fl32  / fullScale_mV;  // g per mV
      updatedConversionFactor_f64 = gramsPerMillivolt;
      updatedConversionFactor_f64 *= 2.0f; // empirically identified
  }
}


// #define LOADCELL_RADING_INTERVALL_IN_US (uint32_t)500
float LoadCell_ADS1220::getReadingKg() const {
  ADS1220_WE& adc = getADC();
  unsigned int timeout_us = 0; //TIMEOUT_FOR_DRDY_TO_BECOME_LOW;
  boolean timeoutReached_b = false;
  static float voltage_mV;

  // wait for the timer to fire
  // This will block until the timer callback gives the semaphore. It won't consume CPU time while waiting.
  if(timer_fireLoadcellReadingReady_global != NULL)
  {
    if (xSemaphoreTake(timer_fireLoadcellReadingReady_global, portMAX_DELAY) == pdTRUE) {
      
      // final check if DRDY is low. If nor, just discard the measurement.
      if (digitalRead(FFB_ADS1220_DRDY) == LOW)
      {
        // Read the voltage from the ADS1220
        voltage_mV = adc.getVoltage_mV();
      }
      
    }
  }
  else
  {
    timer_fireLoadcellReadingReady_global = xSemaphoreCreateBinary();
  }

  float weight_grams = voltage_mV * updatedConversionFactor_f64;
  float weight_kg = weight_grams * 0.001f; // convert grams to kg
  
  // correct bias, assume AWGN --> 3 * sigma is 99.9 %
  return weight_kg - ( _zeroPoint + 3.0f * _standardDeviationEstimate );
}



void LoadCell_ADS1220::estimateBiasAndVariance() {
  getADC(); // Ensure ADC is initialized
  
  Serial.println("Identify loadcell bias and variance");
  float varEstimate;
  float mean = 0.0f;
  float M2 = 0.0f;
  long n = 0;
  

  // capturer N measurements on do regressive mean and variance estimate
  // Use Welford-algorithm
  for (long i = 0; i < NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION; i++){
    float loadcellReading = getReadingKg();
    n++;
    float delta = loadcellReading - mean;
    mean += delta / n;
    M2 += delta * (loadcellReading - mean);
  }

  varEstimate = M2 / ((float)n - 1.0f); // empirical variance 
  // make sure estimate is nonzero
  if (varEstimate < LOADCELL_VARIANCE_MIN) { 
    varEstimate = LOADCELL_VARIANCE_MIN;
  }

  _zeroPoint = mean;
  _standardDeviationEstimate = sqrt(varEstimate);
  _varianceEstimate = varEstimate;

  Serial.print("Offset ");
  Serial.print(_zeroPoint, 5);
  Serial.println("kg");

  Serial.print("Stddev. est.: ");
  Serial.print(_standardDeviationEstimate, 5);
  Serial.println("kg");
}

#endif
