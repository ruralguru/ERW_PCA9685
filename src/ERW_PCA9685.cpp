/* original Adafruit text:
This is a library for our Adafruit 16-channel PWM & Servo driver

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products/815

These displays use I2C to communicate, 2 pins are required to
interface.

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution

Editor: Earl R. Watkins II Title: ERW_PCA9685.h Date: 03/28/2019

	NXP PCA9685 LED Driver Library
	16 seperate outputs. I2C

	Notes:

	Based on: Adafruit library for 16-channel PWM & Servo driver
*/

#include "ERW_PCA9685.h"

/**
 * @brief ERW_PCA9685 Contructor for the PCA9685.
 * @param addr is the I2C address to be used for device.
 * @param LEDsUsed are the number of LED's used.
 */
ERW_PCA9685::ERW_PCA9685(uint8_t addr, uint8_t LEDsUsed)
{
  PCA9685_LEDs_used = LEDsUsed; /* Stores LED count for use in other functions */
  PCA9685_addr = addr; /* Stores I2C address for later */
  PCA9685_clock_frequency = PCA9685_DEFAULT_CLOCK; /* The default internal clock frequency is 25Mhz. */
  PCA9685_I2C = &Wire; /*sets up I2C pointer*/
  PCA9685_device_mode = PCA9685_ALLCALL_BIT | PCA9685_AUTOINC_BIT; /* Sets autoincrement so all 4 registers per LED can be set at once. Also listens on allcall.*/
  PCA9685_n_outputEnable_State = 2; /* Starts in unsettable state to determine if not set */
}

/**
 * @brief ERW_PCA9685 Contructor for the PCA9685.
 * @param addr is the I2C address to be used for device.
 * @param LEDsUsed are the number of LED's used.
 * @param n_outputenable controls the full IC.
 */
ERW_PCA9685::ERW_PCA9685(uint8_t addr, uint8_t LEDsUsed, int n_outputEnable)
{
  PCA9685_n_outputEnable = n_outputEnable; /* Stores OE pin number for use toggling using class function.*/
  PCA9685_LEDs_used = LEDsUsed; /* Stores LED count for use in other functions */
  PCA9685_addr = addr; /* Stores I2C address for later */
  PCA9685_clock_frequency = PCA9685_DEFAULT_CLOCK; /* The default internal clock frequency is 25Mhz. */
  PCA9685_I2C = &Wire;
  PCA9685_device_mode = PCA9685_ALLCALL_BIT | PCA9685_AUTOINC_BIT; /* Sets autoincrement so all 4 registers per LED can be set at once. Also listens on allcall.*/
  pinMode(PCA9685_n_outputEnable, OUTPUT); /* Sets OE pin as an output for use inside class */
  digitalWrite(PCA9685_n_outputEnable, HIGH); /* Sets Pin high to turn off LEDs */
  PCA9685_n_outputEnable_State = 0; /* stores this off state as being off */
}

/**
 * @brief begin does setup and testing I2C interface
 *              Also Turns on auto_increment for faster communication.
 * @return      0 for pass and a -1 for fail.
 */
int8_t ERW_PCA9685::begin(uint8_t drive_mode)
{
  int8_t returnVar = 0;
  uint8_t tempDevice = 0;
  PCA9685_I2C->begin(); /* I2C begin */
  restart();
  PCA9685_device_mode = PCA9685_device_mode | PCA9685_ALLCALL_BIT | PCA9685_AUTOINC_BIT;
  I2C_write(PCA9685_MODE1 ,PCA9685_device_mode);
  tempDevice = PCA9685_device_mode;
  PCA9685_device_mode = I2C_read(PCA9685_MODE1);
  if(PCA9685_device_mode != tempDevice)
  {
    returnVar == -1;
  }

  I2C_write(PCA9685_MODE2 ,drive_mode);
  PCA9685_drive_mode = I2C_read(PCA9685_MODE2);

  if(PCA9685_drive_mode != drive_mode)
  {
    returnVar == -1;
  }
  PCA9685_I2C->beginTransmission(PCA9685_addr);
  PCA9685_I2C->write(ALLLED_ON_L);
  PCA9685_I2C->write(0x00);
  PCA9685_I2C->write(0x00);
  PCA9685_I2C->write(0x00);
  PCA9685_I2C->write(0x10);
  PCA9685_I2C->endTransmission();
  uint8_t tempRead = I2C_read(LED0_OFF_H);
  PCA9685_LED_state = 15;
  if( !( tempRead & 0x10))
  {
    returnVar == -1;
  }
  else
  {
    PCA9685_LED_state = 0;
  }
  #ifdef SERIAL_DEBUG
    Serial.println("Constructor Settings: ");
    Serial.print("\tLEDs used: ");
    Serial.print(PCA9685_LEDs_used);
    Serial.print("\tI2C Address: 0x");
    Serial.print(PCA9685_addr, HEX);
    Serial.print("\tDevice: 0x");
    Serial.print(PCA9685_device_mode, HEX);
    if(PCA9685_n_outputEnable_State == 2)
    {
      Serial.print("\tOE pin: -N/A-");
      Serial.println("\tOE state: -N/A-");
    }
    else
    {
      Serial.print("\tOE pin: ");
      Serial.println(PCA9685_n_outputEnable);
      Serial.print("\tOE state: ");
      Serial.println(PCA9685_n_outputEnable_State);
    }
    Serial.println("Mode Settings: ");
    Serial.print("\tDevice: ");
    Serial.print(tempDevice, HEX);
    Serial.print(" -> ");
    Serial.print(PCA9685_device_mode, HEX);
    Serial.print("\tDrive: ");
    Serial.print(drive_mode, HEX);
    Serial.print(" -> ");
    Serial.print(PCA9685_drive_mode, HEX);
    Serial.print("\tDriver State: ");
    Serial.println(PCA9685_LED_state, HEX);
    Serial.println("Initial LED Reads: ");
    for(uint8_t indexValue = 0; indexValue < PCA9685_LEDs_used; indexValue++)
    {
      Serial.print("\t");
      Serial.print(indexValue);
      Serial.print(": 0x");
      uint8_t tempPrintVal = I2C_read(LED0_ON_L+4*indexValue);
      Serial.print(tempPrintVal, HEX);
      tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 1);
      Serial.print(tempPrintVal, HEX);
      Serial.print(", 0x");
      tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 2);
      Serial.print(tempPrintVal, HEX);
      tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 3);
      Serial.print(tempPrintVal, HEX);
    }
    Serial.println();
  #endif

  return returnVar;
}

/**
 * @brief PMW_freq sets the PWM frequency.
 * @param  desiredFrequency The PWM frequency between 24 Hz and 1526 Hz.
 * @return Returns 0 if success and -1 if the radback doesnt match.
 */
int8_t ERW_PCA9685::PMW_freq(float desiredFrequency)
{
  int8_t returnVar = 0;
  uint8_t holder_prescale;
  desiredFrequency *= 0.9;  // Frequency correction from Adafruit.
  float temp_prescale = PCA9685_clock_frequency;
  temp_prescale /= 4096;
  temp_prescale /= desiredFrequency;
  temp_prescale -= 1;
  uint8_t prescale = floor(temp_prescale + 0.5);
  sleep();
  I2C_write(PCA9685_PRESCALE, prescale);
  restart();
  I2C_write(PCA9685_MODE1, PCA9685_device_mode);  //  This sets the MODE1 register to turn on auto increment.
  PCA9685_device_mode = I2C_read(PCA9685_MODE1);
  holder_prescale = I2C_read(PCA9685_PRESCALE);

  #ifdef SERIAL_DEBUG
    Serial.print("\t Prescaler: ");
    Serial.print(prescale, HEX);
    Serial.print(" -> ");
    Serial.println(holder_prescale, HEX);
  #endif

  if (prescale != holder_prescale)
  {
    returnVar = -1;
  }

  return returnVar;

}

/**
 * @brief set_brightness  The following function takes the desired bightness and stores it.
 *                        The brightness value is capped at 4095 giving full brightness.
 *                        The LED count from the constructor is used to give the LEDs time offsets.
 * @param PCA9685_brightness[16] This is an array of the desired brightnesses.
 */
void ERW_PCA9685::set_brightness(uint16_t (&PCA9685_brightness)[16])
{
  uint8_t high_byte = 0;
  uint8_t low_byte = 0;
  uint16_t temp_offset = 0;
  #ifdef SERIAL_DEBUG
    Serial.println("LED Periods (Start -> Stop)");
  #endif
  for(uint8_t indexValue = 0; indexValue < PCA9685_LEDs_used; indexValue++)
  {
    if( PCA9685_brightness[indexValue] == 0 )
    {
      ALL_LED_ON_OFF_REG[indexValue].LED_ON_H_REG = 0;
      ALL_LED_ON_OFF_REG[indexValue].LED_ON_L_REG = 0;
      ALL_LED_ON_OFF_REG[indexValue].LED_OFF_H_REG = 0x10; /* When the Full Off bit is set no other bit matters */
      ALL_LED_ON_OFF_REG[indexValue].LED_OFF_L_REG = 0;
    }
    else
    {
      if( indexValue == 0)
      {
        temp_offset = 0;
      }
      else
      {
        temp_offset = ( ( PCA9685_DEFAULT_RESOLUTION / PCA9685_LEDs_used ) * indexValue ) - 1;
      }
      split_uint16(temp_offset, high_byte, low_byte);

      high_byte == high_byte & 0x0F;
      if( PCA9685_brightness[indexValue] >= PCA9685_DEFAULT_MAX )
      {

        ALL_LED_ON_OFF_REG[indexValue].LED_ON_H_REG = high_byte | 0x10;  /* When the Full On bit is setonly full off matters */
        ALL_LED_ON_OFF_REG[indexValue].LED_ON_L_REG = low_byte;
        ALL_LED_ON_OFF_REG[indexValue].LED_OFF_H_REG = 0;
        ALL_LED_ON_OFF_REG[indexValue].LED_OFF_L_REG = 0;
      }
      else
      {
        ALL_LED_ON_OFF_REG[indexValue].LED_ON_H_REG = high_byte;
        ALL_LED_ON_OFF_REG[indexValue].LED_ON_L_REG = low_byte;
        temp_offset += PCA9685_brightness[indexValue];
        if( temp_offset >= PCA9685_DEFAULT_MAX)
        {
          temp_offset -= PCA9685_DEFAULT_MAX;
        }
        split_uint16(temp_offset, high_byte, low_byte);
        high_byte = high_byte & 0x0F;
        ALL_LED_ON_OFF_REG[indexValue].LED_OFF_H_REG = high_byte;
        ALL_LED_ON_OFF_REG[indexValue].LED_OFF_L_REG = low_byte;
      }
    }
    #ifdef SERIAL_DEBUG
      Serial.print("\t");
      Serial.print(indexValue);
      Serial.print(": 0x");
      Serial.print(ALL_LED_ON_OFF_REG[indexValue].LED_ON_H_REG, HEX);
      Serial.print(ALL_LED_ON_OFF_REG[indexValue].LED_ON_L_REG, HEX);
      Serial.print("-> 0x");
      Serial.print(ALL_LED_ON_OFF_REG[indexValue].LED_OFF_H_REG, HEX);
      Serial.print(ALL_LED_ON_OFF_REG[indexValue].LED_OFF_L_REG, HEX);
      if(indexValue >= ( PCA9685_LEDs_used - 1 ) )
      {
        Serial.println();
      }
    #endif
  }
}

/**
 * @brief LED_state is used to turn on or off the LEDs using the brightness settings.
 * @param LED_State bitwise state of all 16 available channels.
 */
void ERW_PCA9685::LED_state(uint16_t LED_State)
{
  uint16_t LED_state_changed;
  LED_state_changed = PCA9685_LED_state ^ LED_State;
  PCA9685_LED_state = LED_State;
  #ifdef SERIAL_DEBUG
    Serial.print("LEDs Changed: ");
    Serial.print(LED_state_changed, BIN);
    Serial.println("\tLEDs Set: (index: START -> STOP)");
  #endif
  for(uint8_t indexValue = 0; indexValue < PCA9685_LEDs_used; indexValue++)
  {
    if( bitRead(LED_state_changed, indexValue) )
    {
      if( bitRead(LED_State, indexValue) )
      {
        uint8_t LED_register = LED0_ON_L+4*indexValue;
        PCA9685_I2C->beginTransmission(PCA9685_addr);
        PCA9685_I2C->write(LED_register);
        PCA9685_I2C->write(ALL_LED_ON_OFF_REG[indexValue].LED_ON_L_REG);
        PCA9685_I2C->write(ALL_LED_ON_OFF_REG[indexValue].LED_ON_H_REG);
        PCA9685_I2C->write(ALL_LED_ON_OFF_REG[indexValue].LED_OFF_L_REG);
        PCA9685_I2C->write(ALL_LED_ON_OFF_REG[indexValue].LED_OFF_H_REG);
        PCA9685_I2C->endTransmission();
      }
      else
      {
        uint8_t LED_register = LED0_ON_L+4*indexValue;
        PCA9685_I2C->beginTransmission(PCA9685_addr);
        PCA9685_I2C->write(LED0_ON_L+4*indexValue);
        PCA9685_I2C->write(0);
        PCA9685_I2C->write(0);
        PCA9685_I2C->write(0);
        PCA9685_I2C->write(0x10);
        PCA9685_I2C->endTransmission();
      }
    #ifdef SERIAL_DEBUG
      Serial.print("\t");
      Serial.print(indexValue);
      Serial.print(": 0x");
      uint8_t tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 1);
      Serial.print(tempPrintVal, HEX);
      tempPrintVal = I2C_read(LED0_ON_L+4*indexValue);
      Serial.print(tempPrintVal, HEX);
      Serial.print("-> 0x");
      tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 3);
      Serial.print(tempPrintVal, HEX);
      tempPrintVal = I2C_read( (LED0_ON_L+4*indexValue) + 2);
      Serial.print(tempPrintVal, HEX);
      if( indexValue >= ( PCA9685_LEDs_used - 1) )
      {
        Serial.println();
      }
    #endif
    }
  }
}

/**
 * @brief external_clock sets the device to use external clock.
 *                       Can only be unset with a power cycle.
 *                       TODO: test and flush out.
 * @param ClockFreq Input the Clock Frequency that is used.
 */
void ERW_PCA9685::external_clock(float ClockFreq)
{
  /* TODO: test and flush out */
  uint8_t temp_mode = 0;
  temp_mode = PCA9685_device_mode | PCA9685_EXTCLK_BIT;
  PCA9685_clock_frequency = ClockFreq;
  sleep();
  I2C_write(PCA9685_MODE1, temp_mode);
  restart();
  I2C_write(PCA9685_MODE1, temp_mode);
  PCA9685_device_mode = I2C_read(PCA9685_MODE1);
}

/**
 *@brief restart is used to wake the device from sleep.
 */
void ERW_PCA9685::restart(void)
{
  I2C_write(PCA9685_MODE1, PCA9685_RESTART_BIT);
  delay(10);
}

void ERW_PCA9685::toggleIC(void)
{
  if( PCA9685_n_outputEnable_State )
  {
    digitalWrite(PCA9685_n_outputEnable, HIGH);
    PCA9685_n_outputEnable_State = 0;
  }
  else
  {
    digitalWrite(PCA9685_n_outputEnable, LOW);
    PCA9685_n_outputEnable_State = 1;
  }
  #ifdef SERIAL_DEBUG
    Serial.print("Driver State: ");
    Serial.println(PCA9685_n_outputEnable_State);
  #endif
}

/**
 *@brief sleep is used to put the device to sleep.
 */
void ERW_PCA9685::sleep(void)
{
  uint8_t temp_mode = 0;
  temp_mode =  PCA9685_SLEEP_BIT;
  I2C_write(PCA9685_MODE1, temp_mode);
}

/**
 * @brief The Auto Increment function allows the user to iderate through
 *        registers without needing to state the register.
 */
void ERW_PCA9685::auto_increment(void)
{
  uint8_t temp_mode = 0;
  temp_mode = PCA9685_device_mode & PCA9685_AUTOINC_BIT;
  if( temp_mode )
  {
    temp_mode = PCA9685_device_mode & (~PCA9685_AUTOINC_BIT);
  }
  else
  {
    temp_mode = PCA9685_device_mode | PCA9685_AUTOINC_BIT;
  }
  I2C_write(PCA9685_MODE1, temp_mode);
  PCA9685_device_mode = I2C_read(PCA9685_MODE1);
  #ifdef SERIAL_DEBUG
    if(PCA9685_device_mode & PCA9685_AUTOINC_BIT)
    {
      Serial.println("Auto Increment has been set.");
    }
    else
    {
      Serial.println("Auto Increment has been unset.");
    }
  #endif
}

/**
 * @brief alternative_address_response sets the device's response to the subaddresses and all call address.
 *        TODO: Needs complete testing.
 * @param response_byte is used to set the sub adress responses
 */
int8_t ERW_PCA9685::alternative_address_response(uint8_t response_byte)
{
  int8_t returnVar = 0;
  if (response_byte & 0xF0)
  {
    returnVar = -1;
  }
  else
  {
    PCA9685_device_mode = PCA9685_device_mode | response_byte;
    I2C_write(PCA9685_MODE1, PCA9685_device_mode);
    PCA9685_device_mode = I2C_read(PCA9685_MODE1);
    if ( !( PCA9685_drive_mode & response_byte ) )
    {
      returnVar = -1;
    }
  }
  return returnVar;
}

/**
 * @brief Drive Mode is used to set the drive mode of the device.
 *        See section 7.3.2 of the datasheet for details.
 * @param  mode The Mode is set to drive the LEDs as desired.
 * @return      If the register isn't set as it was requested -1 is returned.
 */
int8_t ERW_PCA9685::drive_mode(uint8_t mode)
{
  int8_t returnVar = 0;
  I2C_write(PCA9685_MODE2, mode);
  PCA9685_drive_mode = I2C_read(PCA9685_MODE2);
  if ( PCA9685_drive_mode != mode )
  {
    returnVar = -1;
  }
  return returnVar;
}

/**
 * @ brief setSerialOut defines the SERIAL_DEBUG state
 * @param state 0 for off 1 for on
 */

/**
 * @brief This function is used to read a register.
 * @param  I2C_reg The register to be read from
 * @return         Value read from the register
 */
uint8_t ERW_PCA9685::I2C_read(uint8_t I2C_reg)
{
  PCA9685_I2C->beginTransmission(PCA9685_addr);
  PCA9685_I2C->write(I2C_reg);
  PCA9685_I2C->endTransmission();
  PCA9685_I2C->requestFrom((uint8_t)PCA9685_addr, (uint8_t)1);
  return PCA9685_I2C->read();
}

/**
 * @brief This function is used to write a register.
 * @param I2C_reg The register desired to be written to.
 * @param data    The byte of data to be written to the register.
 */
void ERW_PCA9685::I2C_write(uint8_t I2C_reg, uint8_t data)
{
  PCA9685_I2C->beginTransmission(PCA9685_addr);
  PCA9685_I2C->write(I2C_reg);
  PCA9685_I2C->write(data);
  PCA9685_I2C->endTransmission();
}

/**
 * @brief The function splits a two byte value into seperate variables.
 * @param toSplit   A two byte value to be split into two bits.
 * @param high_byte [description]
 * @param low_byte  [description]
 */
void ERW_PCA9685::split_uint16(uint16_t toSplit, uint8_t &high_byte, uint8_t &low_byte)
{
  low_byte = uint8_t(toSplit);
  high_byte = uint8_t(toSplit >> 8);
}
