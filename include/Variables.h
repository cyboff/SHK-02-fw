//Variables and definitions used in the project
#ifndef __Variables_H
#define __Variables_H

// TEENSY3.2 pin assignment
#define FILTER_PIN 24 // not connected, for internal use of Bounce2 library filter - SIGNAL PRESENT filter ON/OFF

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN 2 // Serial1: RX1=0 TX1=1 TXEN=2

// SPI
#define SPI_CS 10 // LATCH on AD420

// Define pins for the LED display.
// You can change these, just re-wire your board:
#define LED_DATA_PIN 3         // 3 connects to the display's data in
#define LED_RS_PIN 4           // 4 the display's register select pin
#define LED_CLK_PIN 5          // 5 the display's clock pin
#define LED_EN_PIN  6          // 6 the display's chip enable pin
#define LED_RST_PIN 7          // 7 the display's reset pin

#define DISPLAY_LENGHT 8 // number of characters in the display

// LEDs and I/O

#define LED_POWER 17
#define LED_SIGNAL 18 // now same as OUT_SIGNAL
#define LED_ALARM 19

#define PIN_BTN_A 20 //(digital pin)
#define PIN_BTN_B 21 //(digital pin)
#define PIN_BTN_C 22 //(digital pin)
#define PIN_BTN_D 23 //(digital pin)

#define TEST_IN 28
#define SET_IN 27

#define LASER 26
#define IR_LED 25
#define OUT_SIGNAL_NEG 8
#define OUT_ALARM_NEG 9 //negative output: 24V=>OK, 0V=>ALARM

#define MOTOR_ALARM 14  //pulses from Hall probe
#define MOTOR_ENABLE 15 //enable motor rotation
#define MOTOR_CLK 16    //motor speed clock

// Timeout definitions
#define TIMEOUT_LASER 1200000
#define TIMEOUT_TEST 600000 // 5 min

extern volatile boolean alarmChecked;
extern volatile boolean extTest;
extern volatile boolean intTest;
extern volatile int currentMenu;
extern volatile int currentMenuOption;


extern volatile int hourTimeout;
extern volatile int refreshMenuTimeout;
extern volatile int laserTimeout;
extern volatile int testTimeout;
extern volatile int menuTimeout;

// button interrupt
#define STATE_NORMAL 0
#define STATE_SHORT 1
#define STATE_LONG 2

extern volatile int resultButtonA; // global value set by checkButton()
extern volatile int resultButtonB; // global value set by checkButton()
extern volatile int resultButtonC; // global value set by checkButton()
extern volatile int resultButtonD; // global value set by checkButton()
extern volatile boolean BtnReleasedA;
extern volatile boolean BtnReleasedB;
extern volatile boolean BtnReleasedC;
extern volatile boolean BtnReleasedD;


//////////////// registers of your slave ///////////////////
enum
{
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  ENUM_SIZE,
  MB_MODEL_TYPE,
  MB_MODEL_SERIAL_NUMBER,
  MB_FW_VERSION,

  MODBUS_ID,     // address 1..247
  MODBUS_SPEED,  // baud rate/100 to fit into word
  MODBUS_FORMAT, // SERIAL_8N1 = 0, SERIAL_8E1 = 6, SERIAL_8O1 = 7 , SERIAL_8N2 = 4

  SET,            // MAN1 = 1, MAN2 = 2, RELAY = 3 (REL1 || REL2),
  GAIN_SET1,      // valid values 1,2,4,8,16,32,64 * 100
  THRESHOLD_SET1, // min 20, max 80 * 100
  GAIN_SET2,      // valid values 1,2,4,8,16,32,64 * 100
  THRESHOLD_SET2, // min 20, max 80 * 100

  WINDOW_BEGIN,    // min 5, max 50 * 100
  WINDOW_END,      // min 50 max 95 * 100
  POSITION_MODE,   // rising = 1, falling = 2, peak = 3 , hmd = 4
  ANALOG_OUT_MODE, // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
  POSITION_OFFSET, // min 0, max 2000 

  FILTER_POSITION, // range 0 - 9999 ms (or nr of mirrors) for moving average
  FILTER_ON,       // range 0 - 9999 ms
  FILTER_OFF,      // range 0 - 9999 ms

  ACT_TEMPERATURE,  // act_temp * 256
  MAX_TEMPERATURE,  // max_temp * 256
  TOTAL_RUNTIME,
  IO_STATE,

  PEAK_VALUE,       // 0-100% * 100
  POSITION_VALUE,   // 0-100% * 100
  POSITION_VALUE_AVG,  // 0-100% * 100

  AN_VALUES, // 25 registers
  MOTOR_TIME_DIFF = AN_VALUES + 25,
  EXEC_TIME_ADC,     // exectime of adc conversions
  EXEC_TIME,         // exectime of adc conversions + results calculation
  EXEC_TIME_TRIGGER, // exectime of each triggering
  OFFSET_DELAY,      // calculated trigger delay
  TOTAL_ERRORS,
  // leave this one
  TOTAL_REGS_SIZE
  // total number of registers for function 3 and 16 share the same register array
};

// I/O Status bits for Modbus
enum
{
  IO_LASER,
  IO_IR_LED,
  IO_TEST_IN,
  IO_SET_IN,
  IO_ALARM_OUT,
  IO_SIGNAL_OUT,
  IO_LED_ALARM,
  IO_LED_SIGNAL,
  IO_LED_POWER,
  IO_BTN_A,
  IO_BTN_B,
  IO_BTN_C,
  IO_BTN_D,
  IO_SW_RESET
};

// MODBUS

extern int modbusID;

const uint16_t modbusSpeedArray[] = {12, 48, 96, 192, 384, 576, 1152}; // baudrate/100
extern int actualSpeed;                                                   // array index
extern uint32_t modbusSpeed;            // default 19200

const unsigned int modbusFormatArray[] = {SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, SERIAL_8N2};
extern int actualFormat;
extern unsigned int modbusFormat;

extern boolean dataSent;
extern int sendNextLn;
extern uint16_t io_state;
extern unsigned long exectime;
extern unsigned long pulsetime;

extern uint16_t holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

extern int windowBegin, windowEnd, positionOffset, positionMode, analogOutMode;
extern int filterPosition, filterOn, filterOff;

extern int thre256, thre, thre1, thre2;
extern int set, pga, pga1, pga2;

// diagnosis
extern int celsius; // internal temp in deg of Celsius
extern int temp;    // internal ADC Temp channel value
extern int max_temperature;
extern unsigned int total_runtime;

extern volatile boolean blinkMenu;
extern int setDispIndex;
extern boolean loggedIn;

extern volatile long peakValueTimeDisp;
extern volatile int peakValueDisp;
extern volatile int positionValueDisp;
extern volatile int positionValueAvgDisp;
extern volatile int peakValue;
extern volatile int positionValue;

extern void checkSET();

#endif /* __Variables_H */