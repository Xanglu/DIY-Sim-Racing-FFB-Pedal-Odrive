#include "isv57communication.h"
#include "Main.h"
#include "ODriveUART.h"

ODriveUART odrive(Serial2);

Stream *ActiveSerialForServoCommunication = nullptr;

isv57dynamicStates isv57dynamicStates_;
bool isv57_update_parameter_b = false;
int32_t zeroPos = 0;

isv57communication::isv57communication()
{
    Serial2.begin(115200);
    ActiveSerialForServoCommunication = &Serial2;
    ActiveSerial = ActiveSerialForServoCommunication;
}

void isv57communication::readAllServoParameters()
{
    ODriveFeedback fb = odrive.getFeedback();
    isv57dynamicStates_.servo_pos_given_p = (int32_t)fb.pos;

    // Servo Current as percentage of current limit
    String currentStr = odrive.getParameterAsString("axis0.motor.current_control.Iq_measured");
    float currentA = currentStr.toFloat();
    String currentLimStr = odrive.getParameterAsString("axis0.motor.config.current_lim");
    float currentLimA = currentLimStr.toFloat();
    isv57dynamicStates_.servo_current_percent = (int32_t)((currentA / currentLimA) * 100.0f);


    // Get commanded position error
    String posCmdStr = odrive.getParameterAsString("axis0.controller.pos_setpoint");
    float posCmd = posCmdStr.toFloat();
    float posActual = fb.pos;
    float posError = posCmd - posActual;
    isv57dynamicStates_.servo_pos_error_p = (int32_t)posError;


    // bus voltage
    String vbusStr = odrive.getParameterAsString("vbus_voltage");
    isv57dynamicStates_.servo_voltage_0p1V = (int32_t)(vbusStr.toFloat() * 10.0f);

    int stepsPerMotorRev_u32 = odrive.getParameterAsInt("axis0.encoder.config.cpr");
}

void isv57communication::setupServoStateReading() {
    // For ODrive, no setup is required like iSV57.
    // Reading servo state is done on-demand via getFeedback() or getParameterAsString().
    // This function is kept for compatibility with existing StepperWithLimits calls.
}

void isv57communication::disableAxis()
{
    ActiveSerial->println("Disabling servo axis");
    odrive.setState(AXIS_STATE_IDLE);
}

void isv57communication::enableAxis()
{
    ActiveSerial->println("Enabling servo axis");
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void isv57communication::clearServoUnitPosition() {
    // Reset encoder position, need to investigate 
    odrive.setParameter("axis0.encoder.pos_estimate", "0");

    delay(50);

    float pos = odrive.getPosition();
    isv57dynamicStates_.servo_pos_given_p = (int32_t)pos;

    ActiveSerial->println("ODrive encoder position cleared.");
}

bool isv57communication::setServoVoltage(uint16_t voltageInVolt_u16)
{
    // ODrive doesn’t expose a direct "voltage" setter like iSV57
    // So we just ignore this for now
    (void)voltageInVolt_u16; // avoid unused parameter warning
    return true;
}

bool isv57communication::setPositionSmoothingFactor(uint16_t posSmoothingFactor_u16)
{
    // Pos gain for now, looks like there is something better, config.input_filter_bandwith
    float posGain = static_cast<float>(posSmoothingFactor_u16) / 10.0f; // optional scaling to match ODrive
    odrive.setParameter("axis0.controller.config.pos_gain", posGain);
    return true;
}

bool isv57communication::setRatioOfInertia(uint8_t ratioOfInertia_u8)
{
    // Look into config.interia
    float mappedValue = static_cast<float>(ratioOfInertia_u8); 
    odrive.setParameter("axis0.controller.config.vel_gain", String(mappedValue));
    return true;
}

bool isv57communication::findServosSlaveId()
{
    bool odriveFound = false;

    // Cant do much, just make sure communication to the axis is good
    ODriveFeedback fb = odrive.getFeedback();

    if (fb.pos != 0.0f || fb.vel != 0.0f)
    {
        odriveFound = true;
        ActiveSerial->print("Found ODrive axis 0!\r\n");
    }
    else
    {
        ActiveSerial->print("No ODrive detected on axis 0.\r\n");
    }

    return odriveFound;
}

bool isv57communication::checkCommunication()
{
    ODriveFeedback fb = odrive.getFeedback(); 

    if (!isnan(fb.pos) && !isnan(fb.vel))
    {
        //ActiveSerial->println("Lifeline check: true");
        return true;
    }
    else
    {
        //ActiveSerial->println("Lifeline check: false");
        return false;
    }
}

void isv57communication::setZeroPos()
{
  zeroPos = isv57dynamicStates_.servo_pos_given_p;
}

void isv57communication::applyOfsetToZeroPos(int16_t givenPosOffset_i16)
{
  zeroPos += givenPosOffset_i16;
}

int16_t isv57communication::getZeroPos()
{
  return zeroPos;
}

int16_t isv57communication::getPosFromMin()
{
  return isv57dynamicStates_.servo_pos_given_p - zeroPos;
}


void isv57communication::readServoStates() {
  readAllServoParameters();
int16_t regArray[4];
  // initialize with -1 to indicate non-trustworthyness
  regArray[0] = -1;
  regArray[1] = -1;
  regArray[2] = -1;
  regArray[3] = -1;

  // write to public variables
  // servo_pos_given_p = regArray[0];
  // servo_current_percent = regArray[1];
  // servo_pos_error_p = regArray[2];
  // servo_voltage_0p1V = regArray[3];

  isv57dynamicStates_.servo_pos_given_p = regArray[0];
  isv57dynamicStates_.servo_current_percent = regArray[1];
  isv57dynamicStates_.servo_pos_error_p = regArray[2];
  isv57dynamicStates_.servo_voltage_0p1V = regArray[3];
  isv57dynamicStates_.lastUpdateTimeInMS_u32 = millis();
  
  
  
  if (0)
  {
    ActiveSerial->print("Pos_given:");
    ActiveSerial->print(isv57dynamicStates_.servo_pos_given_p);

    ActiveSerial->print(",Pos_error:");
    ActiveSerial->print(isv57dynamicStates_.servo_pos_error_p);

    ActiveSerial->print(",Cur_given:");
    ActiveSerial->print(isv57dynamicStates_.servo_current_percent);

    ActiveSerial->print(",Voltage:");
    ActiveSerial->print(isv57dynamicStates_.servo_voltage_0p1V);

    ActiveSerial->println(" "); 
  }
  
}


bool isv57communication::clearServoAlarms() {
    odrive.clearErrors();

    return true;
}

bool isv57communication::readCurrentAlarm() {
    ODriveAxisState state = odrive.getState();

    String motorErrorsStr = odrive.getParameterAsString("axis0.motor.error");
    String axisErrorsStr  = odrive.getParameterAsString("axis0.error");

    uint32_t motorErrors = (uint32_t)motorErrorsStr.toInt();
    uint32_t axisErrors  = (uint32_t)axisErrorsStr.toInt();

    ActiveSerial->print("ODrive motor errors: 0x");
    ActiveSerial->println(motorErrors, HEX);

    ActiveSerial->print("ODrive axis errors: 0x");
    ActiveSerial->println(axisErrors, HEX);

    return true;
}

bool isv57communication::readAlarmHistory() {
    ODriveAxisState state = odrive.getState();

    String motorErrorsStr = odrive.getParameterAsString("axis0.motor.error");
    String axisErrorsStr  = odrive.getParameterAsString("axis0.error");

    uint32_t motorErrors = (uint32_t)motorErrorsStr.toInt();
    uint32_t axisErrors  = (uint32_t)axisErrorsStr.toInt();

    ActiveSerial->print("ODrive motor errors: 0x");
    ActiveSerial->println(motorErrors, HEX);

    ActiveSerial->print("ODrive axis errors: 0x");
    ActiveSerial->println(axisErrors, HEX);

    return true;
}

void isv57communication::resetToFactoryParams() 
{
    odrive.setState(AXIS_STATE_IDLE);
    ActiveSerial->println("Rebooting ODrive to reset to factory defaults...");
    
    odrive.setParameter("reboot", "1");

    delay(2000);

    isv57_update_parameter_b = true;
    ActiveSerial->println("ODrive reboot command sent.");
}


void isv57communication::sendTunedServoParameters(bool commandRotationDirection,uint32_t stepsPerMotorRev_u32,uint32_t ratioOfInertia_u32) {
    odrive.setParameter("axis0.encoder.config.cpr", String(stepsPerMotorRev_u32));

    float velGain = static_cast<float>(ratioOfInertia_u32) / 10.0f;
    odrive.setParameter("axis0.controller.config.vel_gain", String(velGain));

    float posGain = 10.0f; // default safe value
    odrive.setParameter("axis0.controller.config.pos_gain", String(posGain));

    if(commandRotationDirection) {
        odrive.setParameter("axis0.motor.config.motor_type", "1"); // Most likely wrong, will look into later
    } else {
        odrive.setParameter("axis0.motor.config.motor_type", "0");
    }

    float currentLimit = 10.0f; // max 10A
    odrive.setParameter("axis0.motor.config.current_lim", String(currentLimit));
    float velLimit = 10000.0f; // encoder units/sec
    odrive.setParameter("axis0.controller.config.vel_limit", String(velLimit));

    odrive.setParameter("axis0.save_configuration", "1");

    ActiveSerial->println("ODrive parameters tuned successfully!");
}
