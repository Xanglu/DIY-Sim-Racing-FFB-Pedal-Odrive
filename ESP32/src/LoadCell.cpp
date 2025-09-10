#include "LoadCell.h"
#include "Main.h"

#ifndef USES_ADS1220

#include <SPI.h>
#include <ADS1256.h>


static const float ADC_CLOCK_MHZ = 7.68f;  // crystal frequency used on ADS1256
static const float ADC_VREF = 2.5f;        // voltage reference

static const int NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION = 1000;
static const float DEFAULT_VARIANCE_ESTIMATE = 0.2f * 0.2f;
static const float LOADCELL_VARIANCE_MIN = 7.0 * 1e-5; // on 8th april 2025, approx. 50g fluctuation were observed --> 6 * sigma = 50g --> sigma = 50g / 6 = 8g --> sigma^2 = (8g)^2 = 0.00064
//static const float CONVERSION_FACTOR = LOADCELL_WEIGHT_RATING_KG / (LOADCELL_EXCITATION_V * (LOADCELL_SENSITIVITY_MV_V/1000));

float updatedConversionFactor_f64 = 1.0f;
#define CONVERSION_FACTOR LOADCELL_WEIGHT_RATING_KG / (LOADCELL_EXCITATION_V * (LOADCELL_SENSITIVITY_MV_V/1000.0f))



uint8_t global_channel0_u8, global_channel1_u8, global_channel2_u8;


// --- Semaphore Handle ---
// Moved to global scope to be accessible by the ISR and the class
static SemaphoreHandle_t drdySemaphore = NULL;

// --- Interrupt Service Routine (ISR) ---
// This function is called every time the DRDY pin goes low
void IRAM_ATTR drdyInterrupt() {
    // Give the semaphore to unblock the reading task.
    if (drdySemaphore != NULL) {
        xSemaphoreGiveFromISR(drdySemaphore, NULL);
    }
}


ADS1256& ADC() {
  static ADS1256 adc(ADC_CLOCK_MHZ, ADC_VREF, /*useresetpin=*/false
  , PIN_DRDY, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);    // RESETPIN is permanently tied to 3.3v


  static bool firstTime = true;
  if (firstTime) {
    ActiveSerial->println("Starting ADC");  
    adc.initSpi(ADC_CLOCK_MHZ);
    delay(1000);
    
    ActiveSerial->println("ADS: send SDATAC command");
    //adc.sendCommand(ADS1256_CMD_SDATAC);
    
    // start the ADS1256 with a certain data rate and gain       
    adc.begin(ADC_SAMPLE_RATE, ADS1256_GAIN_64, false);  
    
    
    ActiveSerial->println("ADC Started");
    
    adc.waitDRDY(); // wait for DRDY to go low before changing multiplexer register
    if ( fabs(CONVERSION_FACTOR) > 0.01f)
    {
        adc.setConversionFactor(CONVERSION_FACTOR);
    }
    else
    {
        adc.setConversionFactor(1);
    }
    firstTime = false;
  }
  return adc;
}


void LoadCell_ADS1256::setLoadcellRating(uint8_t loadcellRating_u8) const {
  ADS1256& adc = ADC();
  float originalConversionFactor_f64 = CONVERSION_FACTOR;
  
  updatedConversionFactor_f64 = 1.0f;
  if (LOADCELL_WEIGHT_RATING_KG>0)
  {
      updatedConversionFactor_f64 = 2.0f * ((float)loadcellRating_u8) * (CONVERSION_FACTOR/LOADCELL_WEIGHT_RATING_KG);
  }
  // ActiveSerial->print("OrigConversionFactor: ");
  // ActiveSerial->print(originalConversionFactor_f64);
  // ActiveSerial->print(",     NewConversionFactor:");
  // ActiveSerial->println(updatedConversionFactor_f64);

  // adc.setConversionFactor( updatedConversionFactor_f64 );
  adc.setConversionFactor( 1 );
}



// --- Class Constructor ---
// This is where we will now handle the one-time setup for the semaphore and interrupt.
LoadCell_ADS1256::LoadCell_ADS1256(uint8_t channel0, uint8_t channel1)
    : _zeroPoint(0.0f), _varianceEstimate(DEFAULT_VARIANCE_ESTIMATE)
{
    global_channel0_u8 = channel0;
    global_channel1_u8 = channel1;

    // --- ONE-TIME SETUP ---
    // Create the binary semaphore ONCE.
    if (drdySemaphore == NULL) {
        drdySemaphore = xSemaphoreCreateBinary();
        if (drdySemaphore != NULL) {
            ActiveSerial->println("DRDY Semaphore created successfully.");
            ActiveSerial->println("starting attach.....");
            // Attach the interrupt ONCE, after the semaphore is created.
            attachInterrupt(digitalPinToInterrupt(PIN_DRDY), drdyInterrupt, FALLING);
            ActiveSerial->println("DRDY interrupt attached.");
        } else {
            ActiveSerial->println("Error: Failed to create DRDY semaphore!");
        }
    }
    
    // Get the ADC instance to ensure it's initialized
    ADS1256& adc = ADC();
    // Set the initial MUX channels for differential reading
    adc.setChannel(channel0, channel1);
}

float LoadCell_ADS1256::getReadingKg() const {
  ADS1256& adc = ADC();

  float weight_kg = 0.0f;
  static float voltage_mV;
  
  // Check if the semaphore is valid before trying to take it.
  if (drdySemaphore != NULL) {
      // Wait for the ISR to give the semaphore.
      // This blocks indefinitely until the DRDY interrupt occurs.
      if (xSemaphoreTake(drdySemaphore, portMAX_DELAY) == pdTRUE) {  

          // additional DRDY check for better smoothness
          // adc.waitDRDY();

          // final check if DRDY is low. If nor, just discard the measurement.
          if (digitalRead(PIN_DRDY) == LOW)
          {
            voltage_mV = adc.readCurrentChannel();
          }
      }
  }

  // Read the value and apply corrections
  // NOTE: The ADC channel is set in the constructor and doesn't need to be set again here
  // unless you are switching between multiple channels in your application.
  weight_kg = voltage_mV * updatedConversionFactor_f64 - (_zeroPoint + 3.0f * _standardDeviationEstimate);

  return weight_kg;
}


void LoadCell_ADS1256::estimateBiasAndVariance() {
  ADS1256& adc = ADC();
  
  ActiveSerial->println("Identify loadcell bias and variance");
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

  ActiveSerial->print("Offset ");
  ActiveSerial->print(_zeroPoint, 5);
  ActiveSerial->println("kg");

  // ActiveSerial->print("Variance est.: ");
  // ActiveSerial->print(varEstimate, 5);
  // ActiveSerial->println("kg");

  ActiveSerial->print("Stddev. est.: ");
  ActiveSerial->print(_standardDeviationEstimate, 5);
  ActiveSerial->println("kg");
}





#endif