#include <Wire.h>
#include <Arduino.h>
#include "Main.h"

uint16_t I2C_Read;
#define I2C_rate 100000
#define I2C_debug_out
//TwoWire I2C_sync(0);
bool I2C_sync_status=false;
bool I2C_data_read=false;
uint16_t I2C_send;
uint32_t iii = 0;

//https://github.com/nickgammon/I2C_Anything/tree/master
template <typename T> unsigned int I2C_writeAnything (const T& value)
  {
  Wire.write((byte *) &value, sizeof (value));
  return sizeof (value);
  }  // end of I2C_writeAnything

template <typename T> unsigned int I2C_readAnything(T& value)
  {
    byte * p = (byte*) &value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++)
          *p++ = Wire.read();
    return i;
  }  // end of I2C_readAnything

void SynconReceive(int len)
{
    /*
    Serial.printf("onReceive[%d]: ", len);
    while (I2C_sync.available()) {
        Serial.write(I2C_sync.read());
    }
    Serial.println();
    */
    if(len>0)
    {
        I2C_readAnything(I2C_Read);        
        I2C_data_read=true;
    }
    
    
    //Serial.println();
    
}
void SynconRequest()
{
    /*
    I2C_sync.print(iii++);
    I2C_sync.print(" Packets.");
    Serial.println("onRequest");
    Serial.println();
    */
   I2C_writeAnything(I2C_send);
    //Wire.write(I2C_send);
}

void I2C_initialize()
{
    //Wire.begin(I2C_SDA,I2C_SCL,I2C_rate);
    delay(3000);
    Serial.println("Sync as Master");
}

void I2C_initialize_slave()
{
    Wire.onReceive(SynconReceive);
    Wire.onRequest(SynconRequest);
    //Wire.begin((uint8_t)I2C_slave_address,I2C_SDA,I2C_SCL,I2C_rate);
    //I2C_sync.begin(I2C_SDA,I2C_SCL,(uint8_t)I2C_slave_address);
    Serial.println("Sync as Slave");
}
