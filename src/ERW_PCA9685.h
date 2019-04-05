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

	This example code uses the "beerware" license. Use it, modify it, and/or reference it.
	If you find it useful, buy me an ice cold tasty beverage someday.
*/

#ifndef ERW_PCA9685_h
#define ERW_PCA9685_h

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>

#define PCA9685_MODE1 		0x00
#define PCA9685_MODE2 		0x01

#define PCA9685_SUBADDR1	0x02
#define PCA9685_SUBADDR2	0x03
#define PCA9685_SUBADDR3	0x04
#define PCA9685_ALLCALL		0x05

#define LED0_ON_L 				0x06
#define LED0_ON_H 				0x07
#define LED0_OFF_L 				0x08
#define LED0_OFF_H 				0x09

#define ALLLED_ON_L 			0xFA
#define ALLLED_ON_H 			0xFB
#define ALLLED_OFF_L 			0xFC
#define ALLLED_OFF_H 			0xFD

#define PCA9685_PRESCALE	0xFE
#define PCA9685_TEST			0xFF

#define PCA9685_LED_FULL_ON		0x1000
#define PCA9685_LED_FULL_OFF	0x1000

#define PCA9685_DEFAULT_CLOCK 25000000
#define PCA9685_DEFAULT_MAX 4095
#define PCA9685_DEFAULT_RESOLUTION 4096

/* The below details are for device mode setings */
#define PCA9685_RESPONSE_BITS 0x0F

#define PCA9685_ALLCALL_BIT		0x01
#define PCA9685_SUBADDR3_BIT	0x02
#define PCA9685_SUBADDR2_BIT	0x04
#define PCA9685_SUBADDR1_BIT	0x08
#define PCA9685_SLEEP_BIT			0x10
#define PCA9685_AUTOINC_BIT		0x20
#define PCA9685_EXTCLK_BIT		0x40
#define PCA9685_RESTART_BIT		0x80

/* The below details are for drive mode setings */
#define PCA9685_OUTNE_BIT			0x03
#define PCA9685_OUTDRV_BIT		0x04
#define PCA9685_OCH_BIT				0x08
#define PCA9685_INVRT_BIT			0x10

//Include needed libraries with ifndef

class ERW_PCA9685
{
	public: //Can be called upon.

	//Public Functions
		ERW_PCA9685(uint8_t addr, uint8_t LEDsUsed);
		int8_t begin(void);
		int8_t PMW_freq(float desiredFrequency);
		void set_brightness(uint16_t (&PCA9685_brightness)[16]);
		void LED_state(uint16_t LED_State);
		void external_clock(float ClockFreq);
		void restart(void);
		void sleep(void);
		int8_t alternative_address_response(uint8_t response_byte);
		int8_t drive_mode(uint8_t mode);
	//Public Variables

	private:
	//Private Functions
	uint8_t I2C_read(uint8_t I2C_reg);
	void I2C_write(uint8_t I2C_reg, uint8_t data);
	void split_uint16(uint16_t toSplit, uint8_t high, uint8_t low);
	void auto_increment(void);
	//Private Variables

		uint8_t PCA9685_addr;
		uint8_t PCA9685_device_mode;
		uint8_t PCA9685_drive_mode;
		uint8_t PCA9685_response_byte;
		float PCA9685_clock_frequency;
		uint8_t PCA9685_alternative_address_response;

		uint8_t PCA9685_LEDs_used;
		uint16_t PCA9685_LED_state;
		TwoWire *PCA9685_I2C;

		struct LED_ON_OFF_REG
	  {
			uint8_t LED_ON_L_REG;
	    uint8_t LED_ON_H_REG;
	    uint8_t LED_OFF_L_REG;
	    uint8_t LED_OFF_H_REG;
	  };

struct LED_ON_OFF_REG ALL_LED_ON_OFF_REG[16];


};
#endif
