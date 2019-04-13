#include <ERW_PCA9685.h>

#define LED_driver_address B01000000  /* I2C address in Binary for the device with all address pins grounded. */
#define LED_driver_Enable 10          /* Sets the enable pin for controlling the toggle functon from the Arduino
                                      NOTE: If multiple devices are ran with one OE_n control the enable can be omitted from the other constructors,
                                      this will cause odd function if the command is called. */
                                      
#define LED_driver_LED_count 16       /* This is the number of LED's driven by the display. This is used to offset the on off times so that the current draw is cleaned up some. */
#define LED_driver_Max 4095           /* This is used to set LEDs to Max Brightness in the for loop */
#define LED_driver_Half 2047          /* This is used to set LEDs to Max Brightness in the for loop */

#define LED_driver_Sixteenth 255

uint16_t LED_driver_brightness[16];                                    /* This declaration makes a brightness array the length needed by the function */
uint8_t LED_driver_DriveMode = PCA9685_OUTNE_BIT | PCA9685_INVRT_BIT;  /* Doesnt Include PCA9685_OCH_BIT 
                                                                                               
NOTES: The OUTNE is is B00000011. Enabling OUTNE_BIT puts output into high Z state when OE is high.
       The OUTDRV_BIT decides totem pole or open-drain. A one corresponds to totem pole.
       The INVRT_BIT decides if the logic states are flipped. Set for when sinking current turns LED on.
       THE OCH_BIT decides if the LED changes after the message or after the ACK
       See datasheet page 29 for INVRT and OUTDRV settings.*/
float LED_driver_Frequency = 1000;
uint16_t LED_driver_state;

ERW_PCA9685 LED_driver(LED_driver_address, LED_driver_LED_count, LED_driver_Enable);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT); //Enable Built in LED.
  digitalWrite(LED_BUILTIN, HIGH); //Turn LED on
  
  Serial.begin(9600); /* this can be changed for faster serial communications */

  /**
  * READ================>   NOTE: If SERIAL_DEBUG is defined the Builtin LED will light.
  * READ================>   No further action is taken until the Serial Terminal is opened.
  */
  #ifdef SERIAL_DEBUG /* This is where the program hangs until the serial port is connected */
    while (!Serial)   /* Delays serial until connection made */
    {
      ;
    }
  #endif
  /* Set up All used LEDs at half duty cycle */
  for (int index = 0; index < LED_driver_LED_count; index++)
  {
    LED_driver_brightness[index] = LED_driver_Half;
  }
  /* setup the IC as desired using above defines */
  LED_driver.begin(LED_driver_DriveMode);
  LED_driver.PMW_freq(LED_driver_Frequency); 
  LED_driver.set_brightness(LED_driver_brightness); /* For ease of transition from off to on the brightness is stored. */
  LED_driver.toggleIC(); /* turn IC on and off with the Pin */
  for(uint8_t loopIndex = 0; loopIndex <= LED_driver_LED_count; loopIndex++ ) /* this  loop turns the LEDs on in a wave */
  {
    bitSet(LED_driver_state, loopIndex);
    LED_driver.LED_state(LED_driver_state);
    delay(100);
  }
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(uint8_t loopIndex = 0; loopIndex <= LED_driver_LED_count; loopIndex++ ) /* this  loop flows the LEDs on in a wave */
  {
    bitSet(LED_driver_state, loopIndex);
    if(loopIndex >= ( LED_driver_LED_count - 1 ) )
    {
      bitClear(LED_driver_state, 0 );      
      bitClear(LED_driver_state, 1 );
    }    
    else if(loopIndex == ( LED_driver_LED_count - 2 ) )
    {
      bitClear(LED_driver_state, ( LED_driver_LED_count - 1 ) );      
      bitClear(LED_driver_state, 0 );
    }
    else
    {
      bitClear(LED_driver_state, ( loopIndex + 1 ) );   
      bitClear(LED_driver_state, ( loopIndex + 2 ) );      
    }
    LED_driver.LED_state(LED_driver_state);
    delay(100);
  }
}
