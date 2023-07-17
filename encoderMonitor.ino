
/* ================================================================================================
 * ESP8266 Encoder Monitoring SW                                                 encoderMonitor.ino                                                  
 * 
 * For in-house use by Pitney Bowes Electricl Engineering Department                    
 * Author: Kevin Harper
 * Date: 05/24/2023
 * Description: This is a program that is intended to be used to monitor a rotary encoders status
 *              when turned by hand (with necessary precautions taken to limit mechanical bouncing
 *              OR physical play in an optical encoder bearing that understandably causes a pitch of 
 *              the incident light shining through the encoder wheel windows via the emitter-detector 
 *              pair). An OLED display is used to show encoder counts. Direction is also grabbed. 
 *              The BRZO I2C library used allows for overclocking of the SSD1306 OLED driver and by 
 *              default disables interrupts during I2C transactions (this was changed in the header). 
 *              The state machine implementation for reading the quadrature signals is taken from 
 *              https://github.com/buxtronix/arduino/tree/master/libraries/Rotary.
 *				The clock interrupt GPIO polling method used for debouncing the "Zero" signal
 *				was adopted from http://www.ganssle.com/debouncing-pt2.htm.
 *
 *				Work is yet to be done fully implementing the battery indicator on the OLED.
 *              
 *              This program uses the following pins:
 *              
 *              Encoder phase A: D6 (ESP8266 GPIO12). Use 4k7 or similar pullup to 3V3.
 *              Encoder phase B: D7 (ESP8266 GPIO13). Use 4k7 or similar pullup to 3V3.
 *              Button pin (zero): D5 (ESP8266 GPIO0). Previous versions used D3 which requires a 
 *								   a particular state on reset
 *              SCL: D1 (ESP8266 GPIO5)
 *              SDA: D2 (ESP8266 GPIO4)
 * ================================================================================================ */
// For WiFi
#include <ESP8266WiFi.h>
#define USING_TIM_DIV256              true            // Default
#include <ESP8266TimerInterrupt.h>
#include "ESP8266_ISR_Timer.h"

// For I2C OLED display
// Made the change in the header file to allow interrupts during I2C writes. Also using v1.3.0
#include <brzo_i2c.h>
#include "SSD1306Brzo.h"

// Preprocessor defined GPIO pin numbers
#define CHA_PIN D7
#define CHB_PIN D6
#define SCL_PIN D1
#define SDA_PIN D2
#define ZERO_PIN D5
#define BATT_PIN A0

// OLED screen info
#define SCREEN_ADDRESS 0x3C   // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH    128   // OLED display width, in pixels
#define SCREEN_HEIGHT    64   // OLED display height, in pixels

// Use the half-step state table (emits a code at 00 and 11), effectively doubles counts
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

// Values returned by checkEncoderState()
#define DIR_NONE 0x0           // No complete step yet.
#define DIR_CW 0x10
#define DIR_CCW 0x20

// Finite state machine implmentation for quadrature encoders
const unsigned char stateTransitionTable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW,  R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,   R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

// Finite state machine implmentation for quadrature encoders
typedef enum {
  BATT_FULL,
  BATT_HALF,
  BATT_LOW,
  BATT_UNDET
} batt_state_t;

// Enum and parallel c-str array for direction indication
enum dir {CW, CCW, UND};
const char* dirNames[3] = {"C.W.", "C.C.W.", "Undet."};
volatile dir directionOfRotation = UND;

// Bitmap image for OLED display. Grabbed using https://javl.github.io/image2cpp/
// Slightly different for using SSD1306/ThingPulse/BRZO vs u8x8 (unbuffered) lib
const byte pbLogoTiles[] = { 
  /*'pb_logo_black', 64x64px*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x80, 0xff, 0xff, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x0f, 0xe0, 0x0f, 0x00, 0x00, 
  0x00, 0x00, 0x7c, 0x00, 0x00, 0x3e, 0x00, 0x00, 
  0x00, 0x00, 0x1f, 0x00, 0x00, 0xf0, 0x00, 0x00, 
  0x00, 0x80, 0x07, 0xe0, 0x07, 0xc0, 0x03, 0x00, 
  0x00, 0xe0, 0x01, 0xff, 0xff, 0x00, 0x07, 0x00, 
  0x00, 0x70, 0xc0, 0x3f, 0xfc, 0x07, 0x0e, 0x00, 
  0x00, 0x38, 0xf0, 0x01, 0x80, 0x1f, 0x1c, 0x00, 
  0x00, 0x1c, 0x7c, 0x00, 0x00, 0x7c, 0x30, 0x00, 
  0x00, 0x0e, 0x1e, 0x80, 0x03, 0xf0, 0x60, 0x00, 
  0x00, 0x07, 0x07, 0xfc, 0x7f, 0xe0, 0xc1, 0x00, 
  0x80, 0xc3, 0x83, 0xff, 0xff, 0x81, 0xc3, 0x01, 
  0x80, 0xc1, 0xc1, 0x1f, 0xf0, 0x07, 0x87, 0x03, 
  0xc0, 0xe1, 0xf0, 0x01, 0x80, 0x1f, 0x0e, 0x03, 
  0xe0, 0x70, 0xf8, 0x00, 0x00, 0x3e, 0x1c, 0x06, 
  0x60, 0x38, 0x3c, 0xf0, 0x1f, 0x7c, 0x18, 0x06, 
  0x70, 0x1c, 0x1e, 0xfe, 0x7f, 0xf0, 0x38, 0x0c, 
  0x30, 0x1c, 0x0f, 0xff, 0xff, 0xe1, 0x70, 0x0c, 
  0x30, 0x0e, 0x87, 0x3f, 0xfc, 0xc1, 0x61, 0x18, 
  0x38, 0x0e, 0x83, 0x07, 0xe0, 0xc1, 0x61, 0x18, 
  0x18, 0x07, 0x01, 0x01, 0x80, 0x00, 0x40, 0x18, 
  0x18, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 
  0x0c, 0xc3, 0xff, 0xff, 0x1f, 0xf0, 0xff, 0x31, 
  0x0c, 0xe3, 0xff, 0xff, 0x7f, 0xfc, 0xff, 0x33, 
  0x8c, 0xe3, 0xff, 0xff, 0x7f, 0xfc, 0xff, 0x33, 
  0x8c, 0xe3, 0xff, 0xff, 0xff, 0xfc, 0xff, 0x33, 
  0x8c, 0x03, 0x00, 0x00, 0xf8, 0x78, 0x00, 0x20, 
  0x8c, 0x01, 0x00, 0x00, 0xf0, 0x78, 0x00, 0x60, 
  0x8c, 0x21, 0x18, 0x06, 0xf0, 0x79, 0x0c, 0x61, 
  0x8c, 0x71, 0x1c, 0x0f, 0xf0, 0x79, 0x8c, 0x63, 
  0x8c, 0x71, 0x3c, 0x0f, 0xf0, 0x79, 0x8c, 0x63, 
  0x8c, 0x71, 0x3c, 0x0f, 0xf0, 0x79, 0x8c, 0x63, 
  0x8c, 0x73, 0x3c, 0x1f, 0xf0, 0x78, 0x8e, 0x61, 
  0x8c, 0xe3, 0x3c, 0x3e, 0xfc, 0x7c, 0x8e, 0x21, 
  0x8c, 0xe3, 0x7c, 0xfe, 0x7f, 0x3c, 0x8e, 0x31, 
  0x8c, 0xe3, 0x78, 0xfc, 0x3f, 0x3e, 0x8e, 0x31, 
  0x0c, 0xe3, 0xf8, 0xf0, 0x1f, 0x1e, 0xc7, 0x31, 
  0x18, 0xc7, 0xf1, 0xe1, 0x07, 0x1f, 0xc7, 0x30, 
  0x18, 0xc7, 0xe3, 0x03, 0xc0, 0x8f, 0xc3, 0x18, 
  0x18, 0x8e, 0xc3, 0x0f, 0xf0, 0x87, 0xe3, 0x18, 
  0x30, 0x8e, 0x87, 0xff, 0xff, 0xc3, 0x61, 0x18, 
  0x30, 0x1c, 0x0f, 0xff, 0xff, 0xe1, 0x71, 0x0c, 
  0x70, 0x1c, 0x1e, 0xfc, 0x7f, 0xf0, 0x38, 0x0c, 
  0x60, 0x38, 0x3c, 0xf0, 0x0f, 0x7c, 0x18, 0x06, 
  0xe0, 0x70, 0xf8, 0x00, 0x00, 0x3e, 0x1c, 0x06, 
  0xc0, 0xe1, 0xf0, 0x03, 0x80, 0x1f, 0x0e, 0x03, 
  0x80, 0xe1, 0xc1, 0x1f, 0xf8, 0x07, 0x87, 0x03, 
  0x80, 0xc3, 0x83, 0xff, 0xff, 0x81, 0x83, 0x01, 
  0x00, 0x87, 0x07, 0xfc, 0x7f, 0xe0, 0xc1, 0x00, 
  0x00, 0x0e, 0x1e, 0xc0, 0x03, 0xf0, 0x60, 0x00, 
  0x00, 0x1c, 0x7c, 0x00, 0x00, 0x7c, 0x70, 0x00, 
  0x00, 0x38, 0xf8, 0x01, 0x80, 0x1f, 0x38, 0x00, 
  0x00, 0x70, 0xe0, 0x7f, 0xfe, 0x07, 0x1e, 0x00, 
  0x00, 0xe0, 0x81, 0xff, 0xff, 0x01, 0x0f, 0x00, 
  0x00, 0xc0, 0x03, 0xf8, 0x3f, 0xc0, 0x03, 0x00, 
  0x00, 0x00, 0x0f, 0x00, 0x00, 0xf0, 0x01, 0x00, 
  0x00, 0x00, 0x7e, 0x00, 0x00, 0x7c, 0x00, 0x00, 
  0x00, 0x00, 0xf8, 0x03, 0x80, 0x1f, 0x00, 0x00, 
  0x00, 0x00, 0xc0, 0xff, 0xff, 0x07, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xfe, 0xff, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const uint8_t lowBatt_icon16x16[32] =
{
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b11111000, 0b00111111, //   ###########   
  0b11111100, 0b01111111, //  #############  
  0b11111110, 0b11111111, // ############### 
  0b00000110, 0b11101100, // ### ##       ## 
  0b00000111, 0b11101100, // ### ##       ###
  0b00000111, 0b11101100, // ### ##       ###
  0b00000110, 0b11101100, // ### ##       ## 
  0b11111110, 0b11111111, // ############### 
  0b11111100, 0b01111111, //  #############  
  0b11111000, 0b00111111, //   ###########   
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};

const uint8_t midBatt_icon16x16[32] = {
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b11111000, 0b00111111, //   ###########   
  0b11111100, 0b01111111, //  #############  
  0b11111110, 0b11111111, // ############### 
  0b10000110, 0b11101101, // ### ## ##    ## 
  0b10000111, 0b11101101, // ### ## ##    ###
  0b10000111, 0b11101101, // ### ## ##    ###
  0b10000110, 0b11101101, // ### ## ##    ## 
  0b11111110, 0b11111111, // ############### 
  0b11111100, 0b01111111, //  #############  
  0b11111000, 0b00111111, //   ###########   
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};

const uint8_t fullBatt_icon16x16[32] = {
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b11111000, 0b00111111, //   ###########   
  0b11111100, 0b01111111, //  #############  
  0b11111110, 0b11111111, // ############### 
  0b10110110, 0b11101101, // ### ## ## ## ## 
  0b10110111, 0b11101101, // ### ## ## ## ###
  0b10110111, 0b11101101, // ### ## ## ## ###
  0b10110110, 0b11101101, // ### ## ## ## ## 
  0b11111110, 0b11111111, // ############### 
  0b11111100, 0b01111111, //  #############  
  0b11111000, 0b00111111, //   ###########   
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
  0b00000000, 0b00000000, //                 
};

// ISR vars declared as volatile
volatile bool buttonFlag = false;
volatile bool phaseEdgeFlag = false;
volatile bool updateCountFlag = true;                   // Init true to update display on reset
volatile bool updateMaxFlag = true;
volatile bool updateMinFlag = true;
volatile signed long int currentMax = 0;
volatile signed long int currentMin = 0;
volatile signed long int counter = 0;

bool updateBatteryFlag = false;

// Global vars
unsigned char state = R_START;                          // Initialize state for encoder state machine
dir currentDirection = CCW;                             // Initialize current direction to CCW...
                                                        // on first rotation, direction updated on the OLED
//For debugging purposes
unsigned int ADC_VAL;
                                                  
// ISR prototypes
void ICACHE_RAM_ATTR encoderISR();
void ICACHE_RAM_ATTR millisecondsTimerHandler();

// Function prototypes
void updateDisplayInfo();
unsigned char checkEncoderState();
void initDisplayUI();
void checkBattVoltage();

// Objects
SSD1306Brzo display(SCREEN_ADDRESS, SDA_PIN, SCL_PIN);  // ADDRESS, SDA, SCL
ESP8266Timer millisecondsTimer;
ESP8266_ISR_Timer secondsTimer;

/* ================================================================================================
 * setup()
 * Return:
 * Description: 
 * Parameters:
 * Locals: 
 * ================================================================================================ */
void setup() {
  // Begin HW serial via USB
  Serial.begin(115200);
  while(!Serial);  

  // Introduce user to Author
  Serial.println(F("\n\nWelcome to ISR based optical motor encoder monitor by Kevin Harper\n"));

  // Init GPIO pins
  pinMode(CHA_PIN, INPUT);
  pinMode(CHB_PIN, INPUT);
  pinMode(ZERO_PIN, INPUT);

  // Attach ISRs to GPIOs
  attachInterrupt(digitalPinToInterrupt(CHA_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHB_PIN), encoderISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ZERO_PIN), buttonHandler, FALLING);

  // Disable WiFi soft AP mode to give ESP more processing power
  WiFi.mode(WIFI_OFF);

  millisecondsTimer.attachInterruptInterval(1000, millisecondsTimerHandler);
  secondsTimer.setInterval(1000, secondsTimerHandler);

  // Initialize the SSSD1306 driver and clear pixels from previous executions
  display.init();
  display.clear();

  checkBattVoltage();
  // Draw PB logo and other static text
  initDisplayUI();

  // Called to init display with undetermined direction info and count of zero
  updateDisplayInfo();
}

/* ================================================================================================
 * loop()
 * Return: void
 * Description: main loop for program execution
 * Parameters: N/A
 * Locals: N/A
 * ================================================================================================ */
void loop() {
  // If either encoder phase had an edge or if the counts were rezeroed...
  if (phaseEdgeFlag) {
    // Update display (this function call inherently has logic built in to prevent unecessary writes
    // to the display buffer. Calls to it may not do anything if none of the appropriate flags are set
    updateDisplayInfo();
    // Reset ISR flags one by one and only as necessary to minimize overhead
    phaseEdgeFlag = false;
  }
  if (buttonFlag) {
    // Reset all counts and obtained information
    counter = 0;
    currentMax = 0;
    currentMin = 0;
    directionOfRotation = UND;
    // Force display to update
    updateCountFlag = true;
    updateMaxFlag = true;
    updateMinFlag = true;
    // Write the update to the display
    updateDisplayInfo();
    buttonFlag = false;
  }
  if (updateBatteryFlag) {
    display.display();
    updateBatteryFlag = false;
  }
}

/* ================================================================================================
 * initDisplayUI()
 * Return: void
 * Description: Writes the PB logo and title to the display
 * Parameters: N/A
 * Locals: N/A
 * ================================================================================================ */
void initDisplayUI() {
  // Keep I2C pin headers aligned with top of OLED frame
  display.flipScreenVertically();

  // Draw a bitmap image
  display.setColor(WHITE);
  display.drawFastImage(0, 0, 64, 64, pbLogoTiles);

  // Setup font for OLED
  display.setFont(ArialMT_Plain_10);
  display.drawString(66, 1, F("Enc Mntr"));
  
  // Draw a line horizontally
  display.drawHorizontalLine(66, 15, display.getStringWidth("Encoder Mntr"));
  display.drawVerticalLine(66, 15, 128);

  // Write static text
  display.drawString(70, 16, F("Cnt: "));
  display.drawString(70, 28, F("Dir: "));
  display.drawString(70, 40, F("Max: "));
  display.drawString(70, 52, F("Min: "));

  // Send the display buffer via I2C
  display.display();
}

/* ================================================================================================
 * updateDisplayInfo()
 * Return: void
 * Description: Writes counts and encoder direction to the OLED display, only if their values change
 * Parameters: N/A
 * Locals: N/A
 * ================================================================================================ */
void updateDisplayInfo() {
  static bool updateDirectionFlag = false;
  // Update direction only if it has changed
  if (currentDirection != directionOfRotation) updateDirectionFlag = true;
  // Added outer conditional in order to call display.display() just once
  if (updateCountFlag || updateDirectionFlag || updateMaxFlag || updateMinFlag) {
    if (updateCountFlag) {
      // Print out a black rectangle over the text to be overwritten
      display.setColor(BLACK);
      display.fillRect(94, 16, 50, 10); // Erase the count
      // Create a c-str from a char buffer representative of counter value
      char counterString[8];
      sprintf(counterString, "%ld", counter);
      // Write the text on the display
      display.setColor(WHITE);
      display.drawString(94, 16, counterString);
      updateCountFlag = false;
    }
    if (updateDirectionFlag) {
      // Print out a black rectangle over the text to be overwritten
      display.setColor(BLACK);
      display.fillRect(94, 28, 50, 10); // Erase the direction
      // Write the text on the display
      display.setColor(WHITE);
      display.drawString(94, 28, dirNames[directionOfRotation]);
      // Update currentDirection to prevent constantly rewriting this entry to the display
      currentDirection = directionOfRotation;
      updateDirectionFlag = false;
    }
    if (updateMaxFlag) {
      display.setColor(BLACK);
      display.fillRect(94, 40, 50, 10); // Erase the max
      char currentMaxString[8];
      sprintf(currentMaxString, "%ld", currentMax);
      display.setColor(WHITE);
      display.drawString(94, 40, currentMaxString);
      updateMaxFlag = false;
    }
    if (updateMinFlag) {
      display.setColor(BLACK);
      display.fillRect(94, 52, 50, 10); // Erase the min
      char currentMinString[8];
      sprintf(currentMinString, "%ld", currentMin);
      display.setColor(WHITE);
      display.drawString(94, 52, currentMinString);
      updateMinFlag = false;
    }
    display.display();
  }
}

/* ================================================================================================
 * checkEncoderState()
 * Return: unsigned char
 * Description: Determines the state of the encoder based on a sequence of A/B quadrature inputs
 * Parameters:
 * Locals: unsigned char pinState
 * ================================================================================================ */
unsigned char checkEncoderState() {
  // Grab state of input pins
  unsigned char pinState = (GPIP(CHA_PIN) << 1) | GPIP(CHB_PIN);
  // Determine new state from the pins and state table
  state = stateTransitionTable[state & 0xf][pinState];
  // Return emit bits, ie the generated event
  return state & 0x30;
}

/* ================================================================================================
 * encoderISR()
 * Return: void
 * Description: ISR for encoder phases; both phase A and phase B invoke this ISR!
 * Parameters: N/A
 * Locals: unsigned char result, return value from checkEncoderState() that is used to determine
 *         whether or not the encoder direction is CW or CCW, and then therefore increment/decrement
 *         the count accordingly
 * ================================================================================================ */
void encoderISR() {
  unsigned char result = checkEncoderState();
  if (result == DIR_CW || result == DIR_CCW) {
    if (result == DIR_CW) {
      directionOfRotation = CW;
      counter++;
    } 
    else if (result == DIR_CCW) {
      directionOfRotation = CCW;
      counter--;
    }
    updateCountFlag = true;
  }
  if (counter > currentMax) {
    currentMax = counter;
    updateMaxFlag = true;
  }
  if (counter < currentMin) {
    currentMin = counter;
    updateMinFlag = true;
  }
  phaseEdgeFlag = true;
}

/* ================================================================================================
 * debounce()
 * Return:
 * Description: 
 * Parameters:
 * Locals: 
 * ================================================================================================ */
bool debounce() {
  static uint16_t state = 0;
  state = (state<<1) | GPIP(ZERO_PIN) | 0xfe00;
  return (state == 0xff00);
}

/* ================================================================================================
 * millisecondsTimerHandler()
 * Return:
 * Description: 
 * Parameters:
 * Locals: 
 * ================================================================================================ */
void millisecondsTimerHandler() {
  static bool started = false;
  static bool buttonState;

  secondsTimer.run();
  
  if (!started) {
    started = true;
  }
  
  buttonState = debounce();
  if (buttonState) buttonFlag = true;
}

/* ================================================================================================
 * secondsTimerHandler()
 * Return:
 * Description: 
 * Parameters:
 * Locals: 
 * ================================================================================================ */
void secondsTimerHandler() {
  static unsigned long int secondsCount = 0;
  if (secondsCount % 2 == 0 && secondsCount != 0) {
    //Serial.println(F("Checking voltage..."));
    checkBattVoltage();
  }
  secondsCount++;
  //Serial.println(secondsCount);
}

/* ================================================================================================
 * checkBattVoltage()
 * Return:
 * Description: 
 * Parameters:
 * Locals: 
 * // need to acount for +-5 counts of noise. once we switch indicators we dont want to oscillate
 * back and forth, so incorporate some sort of loose hysteresis
 * ================================================================================================ */
void checkBattVoltage() {
  static unsigned int batteryVoltage;
  static unsigned int batteryVoltageAverage; 
  static unsigned int state = BATT_HALF;
  static unsigned int temp = BATT_UNDET; // Init to an indeterminant state to force an update
  static unsigned int count = 0;
  static bool first = true;
  
  batteryVoltage = analogRead(BATT_PIN);
  //For debugging purposes
  Serial.print(F("Raw ADC: "));
  Serial.println(batteryVoltage);
  Serial.print("State: ");
  Serial.println(state);
  //ADC_VAL = batteryVoltage;

  batteryVoltageAverage += batteryVoltage;
  count++;

  if (count == 10 || first) {

    Serial.print(F("Sum for averaging: "));
    Serial.println(batteryVoltageAverage);
    
    batteryVoltageAverage = batteryVoltageAverage/count;
    
    Serial.print(F("Averaged ADC: "));
    Serial.println(batteryVoltageAverage);
    
    switch(state) {
      // Battery is nearing fully charged
      case(BATT_FULL):
        // If present state is full and ADC reading drops to below 950, then switch state to half
        if (batteryVoltageAverage < 950) {
          state = BATT_HALF;
        }
        /*
        else if (batteryVoltage < 850) {
          state = BATT_LOW;
        }*/
        else state = BATT_FULL;
        break;
  
      // Battery is half full
      case(BATT_HALF):
        // If present state is half and ADC reading drops to below 850, then switch state to low
        if (batteryVoltageAverage < 850) {
          state = BATT_LOW; 
        }
        // If present state is half and ADC reading rises to 950 or above, then switch state to high
        else if (batteryVoltageAverage >= 950) {
          state = BATT_FULL;
        }
        else state = BATT_HALF;
        break;
  
      // Battery is near empty
      case(BATT_LOW):
        // If present state is low and ADC reading rises to 850 or above, then switch state to half
        if (batteryVoltageAverage >= 850) {
          state = BATT_HALF;
        }
        break;
    }
    if (!first) {
      count = 0;
      batteryVoltageAverage = 0;
    }
    else { 
      first = false;
    }
  }
  
  /*
  if (batteryVoltage <= 1023 && batteryVoltage >= 950) batteryStatus = 2;
  else if (batteryVoltage < 950 && batteryVoltage >= 850) batteryStatus = 1;
  else batteryStatus = 0;
  */
  // If the batteryStatus has changed, only then we must update the display
  if (state != temp) {
    display.setColor(BLACK);
    display.fillRect(112, 0, 16, 14);
    display.setColor(WHITE);

    switch (state) {
      case BATT_FULL:
        display.drawIco16x16(112, 0, fullBatt_icon16x16, false);
        break;
      case BATT_HALF:
        display.drawIco16x16(112, 0, midBatt_icon16x16, false);
        break;
      case BATT_LOW:
        display.drawIco16x16(112, 0, lowBatt_icon16x16, false);
        break;
    }
    temp = state;
    // Tell main loop to write the display buffer when most opportune
    updateBatteryFlag = true;
    Serial.println(F("Batt Stat Updated!"));
  }
}

/* ================================================================================================
 *                                          END FILE
 * ================================================================================================ */
