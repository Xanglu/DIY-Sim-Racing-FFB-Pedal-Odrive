#pragma once
#include <Arduino.h>
#include "ODriveUART.h"

// Replace Serial2 with the UART connected to ODrive
extern ODriveUART odrive;

struct isv57dynamicStates {
    int16_t servo_pos_given_p = 0;
    int16_t servo_pos_error_p = 0;
    int16_t servo_current_percent = 0;
    int16_t servo_voltage_0p1V = 0;
    int16_t estimated_pos_error_i16 = 0;
    unsigned long lastUpdateTimeInMS_u32 = 0;
};

class isv57communication {
public:
    isv57communication();

    // Initialization / Setup
    void setupServoStateReading();
    void sendTunedServoParameters(bool commandRotationDirection, uint32_t stepsPerMotorRev_u32, uint32_t ratioOfInertia_u32);
    void readAllServoParameters();
    void readServoStates();
    bool checkCommunication();
    bool findServosSlaveId();
    bool clearServoAlarms();
    bool readAlarmHistory();
    bool readCurrentAlarm();
    void resetToFactoryParams();
    bool setServoVoltage(uint16_t voltageInVolt_u16);
    bool setPositionSmoothingFactor(uint16_t posSmoothingFactor_u16);
    bool setRatioOfInertia(uint8_t ratiOfInertia_u8);

    void clearServoUnitPosition();
    void disableAxis();
    void enableAxis();

    void setZeroPos();
    void applyOfsetToZeroPos(int16_t givenPosOffset_i16);
    int16_t getZeroPos();
    int16_t getPosFromMin();

    int16_t regArray[4];
    int16_t slaveId = 63; 

    isv57dynamicStates isv57dynamicStates_;
    bool isv57_update_parameter_b = false;

  private:
    int16_t zeroPos = 0;
    int32_t encoderCpr = 8192; // default CPR
    Stream *ActiveSerialForServoCommunication = nullptr;

    bool printProfilingFlag_b = false;

};
