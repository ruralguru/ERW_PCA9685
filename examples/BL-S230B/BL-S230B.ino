#include <ERW_PCA9685.h>

#define AlphaNumeric1_address B01000000
#define AlphaNumeric2_address B01000001
#define AlphaNumeric3_address B01000010
#define AlphaNumeric4_address B01000100
#define AlphaNumeric5_address B01001000
#define AlphaNumeric6_address B01010000
#define AlphaNumericDots_address B01100000

#define AlphaNumeric_ALLCALL_address 0xE0

#define Alphanumeric_Enable 10

#define AlphaNumeric_LED_count 16
#define AlphaNumeric_dot_LED_count 12

#define AlphaNumeric_long 4095;
#define Alphanumeric_short 4095;
#define Alphanumeric_dot 4095;

uint16_t AlphaNumeric_brightness[AlphaNumeric_LED_count];
uint16_t AlphaNumeric_dot_brightness[AlphaNumeric_LED_count];
uint8_t Alphanumeric_DriveMode = PCA9685_OUTNE_BIT | PCA9685_OUTDRV_BIT | PCA9685_INVRT_BIT;/* Doesnt Include PCA9685_OCH_BIT */
float AlphaNumeric_Frequency = 1526;

uint16_t BL_230B_ASCII[256];


void setup() {
  // put your setup code here, to run once:
  /**
   * @brief The following index sets up the values proberly for 
   */
  for(int index = 0; index < AlphaNumeric_LED_count; index++)
  {
    if( index == 2 || index == 4 || index == 7 || index == 12 || index == 14 || index == 17 )
    {
      AlphaNumeric_brightness[index] = AlphaNumeric_long;
    }
    else
    {
      AlphaNumeric_brightness[index] = Alphanumeric_short;
    }
    if( index < AlphaNumeric_dot_LED_count )
    {
      AlphaNumeric_dot_brightness[index] = Alphanumeric_dot;
    }
  }
  /**
   * @brief ...ALLCALL is a constructor that uses the allcall address to set all drivers at once. the constructor is overloaded for ease of use
   *        ...X       is a constructor for display number X for using multiple drivers.
   *        ...Dots is for the dots on alphanumeric displays. All drivers needed for 16 digit alphanumeric display as shown leaving the dots on their own driver.
   */
  ERW_PCA9685 AlphaNumeric_ALLCALL(Alphanumeric_Enable);
  ERW_PCA9685 AlphaNumericDots(AlphaNumeric1_address, AlphaNumeric_LED_count);
  ERW_PCA9685 AlphaNumeric1(AlphaNumeric1_address, AlphaNumeric_LED_count);
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric_ALLCALL.drive_mode(Alphanumeric_DriveMode);
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric_ALLCALL.PMW_freq(AlphaNumeric_Frequency);
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric_ALLCALL.set_brightness(AlphaNumeric_brightness);
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric_ALLCALL.restart();
  AlphaNumeric_ALLCALL.toggleIC();
  delay(1000);
  AlphaNumeric_ALLCALL.toggleIC();
  AlphaNumeric1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
