//firmware for SHK-02 board

#include <Arduino.h>

//for display
#include <LedDisplay.h>
#include <stdio.h>
#include <stdarg.h>

//for EEPROM
#include <EEPROM.h>

// for ADC
#include <ADC.h>
#include <DMAChannel.h>

// #include "RingBufferDMA.h"

// for SPI
#include <SPI.h>

// for filters
#include <Bounce2.h>

//for ModBus
#include <SimpleModbusSlave.h>

//for offset delay
#include <TeensyDelay.h>

//defaults EEPROM
#define MODEL_TYPE 50
#define MODEL_SERIAL_NUMBER 22001
#define FW_VERSION 404

#define DEFAULT_MODBUS_ID MODEL_SERIAL_NUMBER % 1000 % 247 // MODBUS ID slave (range 1..247)
#define DEFAULT_MODBUS_SPEED 19200
#define DEFAULT_MODBUS_FORMAT SERIAL_8N1

#define DEFAULT_SET 0             // RELAY = 0 (REL1 || REL2), MAN1 = 1, MAN2 = 2
#define DEFAULT_GAIN_SET1 16      // valid values 1,2,4,8,16,32,64
#define DEFAULT_THRESHOLD_SET1 50 // min 20, max 80
#define DEFAULT_GAIN_SET2 32
#define DEFAULT_THRESHOLD_SET2 50

#if MODEL_TYPE == 10
#define DEFAULT_WINDOW_BEGIN 45 // min 5, max 45
#define DEFAULT_WINDOW_END 55   // min 55 max 95
#elif MODEL_TYPE == 30
#define DEFAULT_WINDOW_BEGIN 35 // min 5, max 45
#define DEFAULT_WINDOW_END 65   // min 55 max 95
#else
#define DEFAULT_WINDOW_BEGIN 20 // min 5, max 45
#define DEFAULT_WINDOW_END 80   // min 55 max 95
#endif

#define DEFAULT_POSITION_MODE 1     // hmd = 0, rising = 1, falling = 2, peak = 3
#define DEFAULT_ANALOG_OUT_MODE 0   // an1/an2: "1Int2Pos" = 0, "1Pos2Int2" = 1, "1Int2Int" = 2, "1Pos2Pos" = 3
#define DEFAULT_POSITION_OFFSET 250 // min 5, max 95 to avoid coincidence with pulse interrupts

#define DEFAULT_FILTER_POSITION 6 // range 0 - 9999 ms (or nr of mirrors) for moving average
#define DEFAULT_FILTER_ON 0       // range 0 - 9999 ms
#define DEFAULT_FILTER_OFF 0      // range 0 - 9999 ms

// EEPROM Addresses (all values are WORD for easy Modbus transfers)

// EEPROM Addresses for signature code and version of firmware
#define EE_ADDR_MODEL_TYPE 0x00          // WORD
#define EE_ADDR_MODEL_SERIAL_NUMBER 0x02 // WORD
#define EE_ADDR_FW_VERSION 0x04          // WORD

// EEPROM Addresses for config
#define EE_ADDR_modbus_ID 0x06     // WORD
#define EE_ADDR_modbus_Speed 0x08  // WORD  // baudrate/100 to fit 115200 to WORD
#define EE_ADDR_modbus_Format 0x10 // WORD

#define EE_ADDR_set 0x12            // WORD  // RELAY = 0 (REL1 || REL2), MAN1 = 1, MAN2 = 2
#define EE_ADDR_gain_set1 0x14      // WORD
#define EE_ADDR_threshold_set1 0x16 // WORD
#define EE_ADDR_gain_set2 0x18      // WORD
#define EE_ADDR_threshold_set2 0x20 // WORD

#define EE_ADDR_window_begin 0x22    // WORD
#define EE_ADDR_window_end 0x24      // WORD
#define EE_ADDR_position_mode 0x26   // WORD  // positionMode: HMD = 0, RISE = 1, FALL = 2, PEAK = 3
#define EE_ADDR_analog_out_mode 0x28 // WORD  // an1/an2: "1Int2Pos" = 0, "1Pos2Int" = 1, "1Int2Int" = 2, "1Pos2Pos" = 3
#define EE_ADDR_position_offset 0x30 // WORD  // offset for position

#define EE_ADDR_filter_position 0x32 // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_on 0x34       // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_off 0x36      // WORD  // range 0 - 9999 ms

// EEPROM Addresses for diagnosis
#define EE_ADDR_max_temperature 0x38 // WORD
#define EE_ADDR_total_runtime 0x40   // WORD

// Define pins
// filters

#define FILTER_PIN 24 // not connected, for internal use of Bounce2 library filter - SIGNAL PRESENT filter ON/OFF

Bounce filterOnOff = Bounce();
float sma = 0;

// Modbus - RS485

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN 2 // Serial1: RX1=0 TX1=1 TXEN=2

// SPI
#define SPI_CS 10 // LATCH on AD420

// Define pins for the LED display.
// You can change these, just re-wire your board:
#define dataPin 7           // 7 connects to the display's data in
#define registerSelectPin 6 // 6 the display's register select pin
#define clockPin 5          // 5 the display's clock pin
#define enablePin 4         // 4 the display's chip enable pin
#define resetPin 3          // 3 the display's reset pin

#define displayLength 8 // number of characters in the display

// create am instance of the LED display library:
LedDisplay myDisplay = LedDisplay(dataPin, registerSelectPin, clockPin, enablePin, resetPin, displayLength);
int brightness = 10; // screen brightness

// LEDs and I/O

#define LED_POWER 19
#define LED_SIGNAL 18 // now same as OUT_SIGNAL
#define LED_ALARM 17

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

// Keycodes
#define BTN_NONE 0 // No keys pressed
#define BTN_A 1    // Button A was pressed
#define BTN_B 2    // Button B was pressed
#define BTN_C 3    // Button B was pressed
#define BTN_D 4    // Button B was pressed
#define BTN_AH 5   // Button A was pressed and holded (BTN_HOLD_TIME) milisecons
#define BTN_BH 6   // Button A was pressed and holded (BTN_HOLD_TIME) milisecons
#define BTN_CH 7   // Button A was pressed and holded (BTN_HOLD_TIME) milisecons
#define BTN_DH 8   // Button A was pressed and holded (BTN_HOLD_TIME) milisecons

// Keyboard times
#define BTN_DEBOUNCE_TIME 200 // debounce time (*500us) to prevent flickering when pressing or releasing the button
#define BTN_HOLD_TIME 3000    // holding period (*500us) how long to wait for press+hold event

#define TIMEOUT_REFRESH_MENU 500
#define TIMEOUT_MENU 1200000 // *500us = 10 mins
#define TIMEOUT_LASER 1200000
#define TIMEOUT_TEST 600000 // 5 min

// display menu
#define MENU_MAIN 1
#define MENU_ALARM 11

#define MENU_LOGIN 12 // PIN  2314

#define MENU_SETUP 2
#define MENU_SENSOR 21
#define MENU_GAIN1 211
#define MENU_THRE1 212
#define MENU_GAIN2 213
#define MENU_THRE2 214
#define MENU_SET 215

#define MENU_MODBUS 22
#define MENU_MODBUS_ID 221
#define MENU_MODBUS_SPEED 222
#define MENU_MODBUS_FORMAT 223

#define MENU_FILTERS 23
#define MENU_FILTER_POSITION 231
#define MENU_FILTER_ON 232
#define MENU_FILTER_OFF 233

#define MENU_ANALOG 24
#define MENU_WINDOW_BEGIN 241
#define MENU_WINDOW_END 242
#define MENU_POSITION_MODE 243
#define MENU_ANALOG_OUT_MODE 244
#define MENU_POSITION_OFFSET 245

#define MENU_INFO 25
#define MENU_RESET 251

// Menu variables
volatile char lastKey = BTN_NONE; // Last key pressed
volatile int hourTimeout = 3600000;
volatile int refreshMenuTimeout = 0;
volatile int laserTimeout = 0;
volatile int testTimeout = 0;
volatile int menuTimeout = 0;
volatile boolean blinkMenu = false;
volatile boolean alarmChecked = false;
volatile boolean extTest = false;
volatile boolean intTest = false;
volatile int currentMenu = MENU_MAIN;
volatile int currentMenuOption = 0;
volatile int btnHoldCounter = 0;

// configure ADC

#define ADC0_AVERAGING 1
#define ANALOG_BUFFER_SIZE 200
unsigned int freq = 400000;

ADC *adc = new ADC();                                 // adc object
volatile DMAMEM int16_t adc0_buf[ANALOG_BUFFER_SIZE]; // buffer 1...
volatile uint8_t adc0_busy = 0;
DMAChannel adc0_dma;
// References for ISRs...
//extern void adc0_dma_isr(void);

volatile int adc_data[ANALOG_BUFFER_SIZE]; // ADC_0 9-bit resolution for differential - sign + 8 bit
volatile int value_buffer[25];
//volatile int value_peak[ANALOG_BUFFER_SIZE];
volatile int adc0Value = 0;         //analog value
volatile int analogBufferIndex = 0; //analog buffer pointer
volatile int delayOffset = 0;
// sensor variables
volatile int thre256 = 75, thre = 30, thre1 = 30, thre2 = 50;
volatile int hmdThresholdHyst = 13;
volatile int pga = 16, pga1 = 16, pga2 = 32;

volatile int windowBegin, windowEnd, positionOffset, positionMode, analogOutMode;
volatile int filterPosition, filterOn, filterOff;

int menu_windowBegin, menu_windowEnd, menu_positionOffset, menu_positionMode, menu_analogOutMode;
int menu_filterPosition, menu_filterOn, menu_filterOff;
const char *menu_positionModeDisp[] = {" HMD", "RISE", "FALL", "PEAK"};
const char *menu_analogOutModeDisp[] = {"1I2P", "1P2I", "1I2I", "1P2P"};

int set = DEFAULT_SET;
const char *menu_setDisp[] = {"REL ", "REL1", "REL2", "MAN1", "MAN2"};
int setDispIndex = 0;
int menu_pga;
int menu_thre;
int menu_set;

// MODBUS

int modbusID = 1;
int menu_modbusID = 1;

const uint16_t modbusSpeedArray[] = {12, 48, 96, 192, 384, 576, 1152}; // baudrate/100
int actualSpeed = 3;                                                   // array index
uint32_t modbusSpeed = modbusSpeedArray[actualSpeed] * 100;            // default 19200
uint32_t menu_modbusSpeed = modbusSpeedArray[actualSpeed] * 100;

const unsigned int modbusFormatArray[] = {SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, SERIAL_8N2};
int actualFormat = 1;
unsigned int modbusFormat = modbusFormatArray[actualFormat];
unsigned int menu_modbusFormat = modbusFormatArray[actualFormat];
const char *menu_modbusFormatDisp[] = {"8N1", "8E1", "8O1", "8N2"};

volatile boolean dataSent = false;
int sendNextLn = 0;
uint16_t io_state = 0;
unsigned long exectime = 0;
unsigned long pulsetime = 0;

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

  SET,            // RELAY = 0 (REL1 || REL2), MAN1 = 1, MAN2 = 2
  GAIN_SET1,      // valid values 1,2,4,8,16,32,64
  THRESHOLD_SET1, // min 20, max 80
  GAIN_SET2,      // valid values 1,2,4,8,16,32,64
  THRESHOLD_SET2, // min 20, max 80

  WINDOW_BEGIN,    // min 5, max 50
  WINDOW_END,      // min 50 max 95
  POSITION_MODE,   // hmd = 0, rising = 1, falling = 2, peak = 3
  ANALOG_OUT_MODE, // an1/an2: "1Int2Pos" = 0, "1Pos2Int" = 1, "1Int2Int" = 2, "1Pos2Pos" = 3
  POSITION_OFFSET, // min 5, max 95 to avoid coincidence with pulse interrupts

  FILTER_POSITION, // range 0 - 9999 ms (or nr of mirrors) for moving average
  FILTER_ON,       // range 0 - 9999 ms
  FILTER_OFF,      // range 0 - 9999 ms

  ACT_TEMPERATURE,
  MAX_TEMPERATURE,
  TOTAL_RUNTIME,
  IO_STATE,

  PEAK_VALUE,
  POSITION_VALUE,
  POSITION_VALUE_AVG,

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

uint16_t holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

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
  IO_BTN_D
};

// Timers

IntervalTimer timer500us; // timer for motor speed and various timeouts

int startTimerValue0 = 0;

volatile int motorPulseIndex = 0;
volatile long motorTimeOld = 0;
volatile long motorTimeNow = 0;
volatile int motorTimeDiff = 0;

volatile long peakValueTimeDisp = 0;
volatile int peakValueDisp = 0;
volatile int positionValueDisp = 0;
volatile int positionValueAvgDisp = 0;

// button interrupt
#define STATE_NORMAL 0
#define STATE_SHORT 1
#define STATE_LONG 2

volatile int resultButtonA = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonB = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonC = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonD = STATE_NORMAL; // global value set by checkButton()
volatile int BtnPressedATimeout = 0;
volatile int BtnPressedBTimeout = 0;
volatile int BtnPressedCTimeout = 0;
volatile int BtnPressedDTimeout = 0;
volatile boolean BtnReleasedA = true;
volatile boolean BtnReleasedB = true;
volatile boolean BtnReleasedC = true;
volatile boolean BtnReleasedD = true;

//menu login
int passwd = 0; // correct passwd is 1122
int nextBtn = 0;
boolean loggedIn = false;

// diagnosis
int celsius; // internal temp in deg of Celsius
int temp;    // internal ADC Temp channel value
int max_temperature = 0;
unsigned int total_runtime = 0;

// Display print wrapper
void displayPrint(const char *format, ...);
void displayMenu(void);
void showAlarm(void);
// Main Menu View
void showMainMenu(void);
void showLoginMenu(void);
void showSetupMenu(void);
void showSensorMenu(void);
void setGain1Menu(void);
void setThre1Menu(void);
void setGain2Menu(void);
void setThre2Menu(void);
void setSetMenu(void);
void showModbusMenu(void);
void setModbusID(void);
void setModbusSpeed(void);
void setModbusFormat(void);
void showFiltersMenu(void);
void setFilterPosition(void);
void setFilterOn(void);
void setFilterOff(void);
void showAnalogMenu(void);
void setWindowBegin(void);
void setWindowEnd(void);
void setPositionMode(void);
void setAnalogOutMode(void);
void setPositionOffset(void);
void showInfoMenu(void);
void showResetMenu(void);

// EEPROM

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(unsigned int address, unsigned int value);
void eeprom_updateInt(unsigned int address, unsigned int value);
// read a unsigned int (two bytes) value from eeprom
unsigned int eeprom_readInt(unsigned int address);
void EEPROM_init();
void config_loadFromEEPROM();
void config_writeDefaultsToEEPROM();
void reset_writeDefaultsToEEPROM();

// check SET and load proper settings
void checkSET();
void checkTEST();
void checkALARM();

// SPI send 2 x 16 bit value
void updateSPI(int valueAN1, int valueAN2);

// INTERRUPT ROUTINES

// button interrupts
//*****************************************************************
void checkButtonA();
void checkButtonB();
void checkButtonC();
void checkButtonD();

// Timer interrupts
void timer500us_isr(void);

// motor (from HALL sensor) interrupt
void motor_isr(void);

void callback_delay();
void adc0_dma_isr(void);
void updateResults();

// exponential moving average
long approxSimpleMovingAverage(int new_value, int period);

void checkSTATUS();
void checkModbus();

void setup()
{
  // initialize LEDs and I/O

  //pinMode(LED_BUILTIN, OUTPUT); //conflicts with SPI_CLK!!!
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_SIGNAL, OUTPUT);
  pinMode(LED_ALARM, OUTPUT);

  pinMode(OUT_SIGNAL_NEG, OUTPUT);
  pinMode(OUT_ALARM_NEG, OUTPUT);

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_SIGNAL, LOW); // connected as OUT_SIGNAL
  digitalWrite(OUT_SIGNAL_NEG, HIGH);
  digitalWrite(LED_ALARM, HIGH);
  digitalWrite(OUT_ALARM_NEG, LOW); // opposite to LED_ALARM

  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  pinMode(PIN_BTN_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), checkButtonA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_B), checkButtonB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_C), checkButtonC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_D), checkButtonD, CHANGE);

  pinMode(TEST_IN, INPUT_PULLUP);
  pinMode(SET_IN, INPUT_PULLUP);

  pinMode(LASER, OUTPUT);
  pinMode(IR_LED, OUTPUT);

  // initialize the LED display library:
  myDisplay.begin();
  // set the brightness of the display:
  myDisplay.setBrightness(brightness);

  // use wrapper for myDisplay.print
  displayPrint("Starting");
  delay(500);

  EEPROM_init();

  //initialize filters
  pinMode(FILTER_PIN, OUTPUT);
  digitalWriteFast(FILTER_PIN, LOW);
  filterOnOff.attach(FILTER_PIN);
  filterOnOff.interval(filterOn);

  //initialize SPI

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPI.begin();
  updateSPI(0, 0);

  // enable serial communication

  holdingRegs[ENUM_SIZE] = TOTAL_REGS_SIZE;

  holdingRegs[MB_MODEL_TYPE] = MODEL_TYPE;
  holdingRegs[MB_MODEL_SERIAL_NUMBER] = MODEL_SERIAL_NUMBER;
  holdingRegs[MB_FW_VERSION] = FW_VERSION;

  holdingRegs[EXEC_TIME] = 0;

  //Serial.begin(modbusSpeed);

  modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);

  //initialize ADC

  ///// ADC0 in differential mode with PGA for signal from photodiode ////

  pinMode(A10, INPUT); // analog input P differential for PGA
  pinMode(A11, INPUT); // analog input N differential for PGA

  adc->adc0->setAveraging(ADC0_AVERAGING); // set number of averages
  adc->adc0->setResolution(9);             // set bits of resolution - 9 bit for differential

  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  //adc->adc0->setReference(ADC_REFERENCE::REF_1V2); // use default 3.3V for input signal > 1.2V

  adc->adc0->enablePGA(pga);

  // Lets setup Analog 0 dma
  adc0_dma.source((volatile uint16_t &)ADC0_RA);
  adc0_dma.destinationBuffer(adc0_buf, sizeof(adc0_buf));
  adc0_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  adc0_dma.interruptAtCompletion();
  adc0_dma.disableOnCompletion();
  adc0_dma.attachInterrupt(&adc0_dma_isr);

  // ADC1 for internal temperature measurement

  adc->adc1->setAveraging(32);                                         // set number of averages
  adc->adc1->setResolution(12);                                        // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);     // change the sampling speed

  // read once for setup everything
  //adc->analogReadDifferential(A10, A11, ADC_0);             //start ADC_0 differential
  adc->analogRead(ADC_INTERNAL_SOURCE::TEMP_SENSOR, ADC_1); //start ADC_1 single, temp sensor internal pin 38

  // adc->enableInterrupts(ADC_0);
  //adc->startContinuousDifferential(A10, A11, ADC_0);
  adc->startContinuous(38, ADC_1); // do not want to accept ADC_INTERNAL_SOURCE::TEMP_SENSOR

  //motor configuration and startup
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW); //motor enable
  pinMode(MOTOR_CLK, OUTPUT);      //motor output pulses

  pinMode(MOTOR_ALARM, INPUT_PULLUP); //motor input pulses
  NVIC_SET_PRIORITY(IRQ_PORTD, 16);   // pin 14 - PortD, see schematic of Teensy 3.2
  if (positionOffset < 1000)          //depends on motor HALL sensors & mirror position - choose the best to have no timing issues
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, FALLING);
  else
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, RISING);

  //NVIC_SET_PRIORITY(IRQ_PORTC, 0);

  //motor slow start
  timer500us.priority(0);

  for (int speed = 20; speed <= 100; speed++)
  {
    timer500us.end();
    startTimerValue0 = timer500us.begin(timer500us_isr, 50000 / speed); //motor output pulses slowly going to 500us
    displayPrint("Mot=%3d%%", speed);
    delay(100);
  }

  TeensyDelay::begin();
  TeensyDelay::addDelayChannel(callback_delay, 0); //setup channel 0

  //clear data buffers
  memset((void *)adc0_buf, 0, sizeof(adc0_buf));
  memset((void *)adc_data, 0, sizeof(adc_data));
}

void loop()
{
  // check SET
  checkSET();
  // check TEST
  checkTEST();
  // check ALARMS and WARNINGS
  checkALARM();

  // check IO STATUS
  checkSTATUS();

  // update modbus
  checkModbus();

  //show info on LED display
  displayMenu();
}

// Display print wrapper
void displayPrint(const char *format, ...)
{
  char S[10];
  va_list arg;
  va_start(arg, format);
  vsnprintf(S, sizeof(S), format, arg);
  va_end(arg);
  myDisplay.home();
  myDisplay.print(S);
}

//***************************************************************************************
// MENUS

void displayMenu(void)
{
  if (!refreshMenuTimeout)
  {

    // button interrupt check

    switch (resultButtonA | resultButtonB | resultButtonC | resultButtonD)
    {

    case STATE_NORMAL:
    {
      lastKey = BTN_NONE;
      break;
    }

    case STATE_SHORT:
    {
      if (resultButtonA == STATE_SHORT)
      {
        lastKey = BTN_A;
      }
      else if (resultButtonB == STATE_SHORT)
      {
        lastKey = BTN_B;
      }
      else if (resultButtonC == STATE_SHORT)
      {
        lastKey = BTN_C;
      }
      else
      {
        lastKey = BTN_D;
      }
      resultButtonA = STATE_NORMAL;
      resultButtonB = STATE_NORMAL;
      resultButtonC = STATE_NORMAL;
      resultButtonD = STATE_NORMAL;
      break;
    }

    case STATE_LONG:
    case 3:
    {
      if (resultButtonA == STATE_LONG)
      {
        lastKey = BTN_AH;
        resultButtonA = STATE_NORMAL;
      }
      else if (resultButtonB == STATE_LONG)
      {
        lastKey = BTN_BH;
        btnHoldCounter++;
        if (BtnReleasedB)
        {
          resultButtonB = STATE_NORMAL;
          btnHoldCounter = 0;
        }
      }
      else if (resultButtonC == STATE_LONG)
      {
        lastKey = BTN_CH;
        btnHoldCounter++;
        if (BtnReleasedC)
        {
          resultButtonC = STATE_NORMAL;
          btnHoldCounter = 0;
        }
      }
      else
      {
        lastKey = BTN_DH;
        resultButtonD = STATE_NORMAL;
      }

      break;
    default:
      break;
    }
    }

    switch (currentMenu)
    {
    case MENU_ALARM:
      showAlarm();
      break;
    case MENU_MAIN:
      showMainMenu();
      break;
    case MENU_LOGIN:
      showLoginMenu();
      break;
    case MENU_SETUP:
      showSetupMenu();
      break;
    case MENU_SENSOR:
      showSensorMenu();
      break;
    case MENU_GAIN1:
      setGain1Menu();
      break;
    case MENU_THRE1:
      setThre1Menu();
      break;
    case MENU_GAIN2:
      setGain2Menu();
      break;
    case MENU_THRE2:
      setThre2Menu();
      break;
    case MENU_SET:
      setSetMenu();
      break;
    case MENU_MODBUS:
      showModbusMenu();
      break;
    case MENU_MODBUS_ID:
      setModbusID();
      break;
    case MENU_MODBUS_SPEED:
      setModbusSpeed();
      break;
    case MENU_MODBUS_FORMAT:
      setModbusFormat();
      break;
    case MENU_FILTERS:
      showFiltersMenu();
      break;
    case MENU_FILTER_POSITION:
      setFilterPosition();
      break;
    case MENU_FILTER_ON:
      setFilterOn();
      break;
    case MENU_FILTER_OFF:
      setFilterOff();
      break;
    case MENU_ANALOG:
      showAnalogMenu();
      break;
    case MENU_WINDOW_BEGIN:
      setWindowBegin();
      break;
    case MENU_WINDOW_END:
      setWindowEnd();
      break;
    case MENU_POSITION_MODE:
      setPositionMode();
      break;
    case MENU_ANALOG_OUT_MODE:
      setAnalogOutMode();
      break;
    case MENU_POSITION_OFFSET:
      setPositionOffset();
      break;
    case MENU_INFO:
      showInfoMenu();
      break;
    case MENU_RESET:
      showResetMenu();
      break;
    default: //showMainMenu();
      break;
    }
    refreshMenuTimeout = TIMEOUT_REFRESH_MENU;
    blinkMenu = !blinkMenu;
    //blink LED_POWER
    digitalWriteFast(LED_POWER, !digitalReadFast(LED_POWER));
  }
}

void showAlarm(void)
{

  if (currentMenuOption == 0)
  {
    if (blinkMenu)
      displayPrint("ALARM!!!");
    else
      displayPrint("MOTORerr");
  }
  if (currentMenuOption == 1)
  {
    if (blinkMenu)
      displayPrint("ALARM!!!");
    else
      displayPrint("Temp %2dC", celsius);
  }
  if (currentMenuOption == 2)
  {
    if (blinkMenu)
      displayPrint("WARNING!");
    else
      displayPrint("EXT_TEST");
  }
  if (currentMenuOption == 3)
  {
    if (blinkMenu)
      displayPrint("WARNING!");
    else
      displayPrint("INT_TEST");
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    currentMenu = MENU_MAIN;
    currentMenuOption = 0;
    alarmChecked = true;
    if (!menuTimeout)
      menuTimeout = TIMEOUT_MENU;
  }
}

// Main Menu View
void showMainMenu(void)
{

  if (currentMenuOption == 0)
    displayPrint("Int %3d%%", peakValueDisp);
  else if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 1)
  {
    displayPrint("Pos %3d%%", positionValueAvgDisp / 10);
  }
  if (currentMenuOption == 2)
    displayPrint("Gain %2dx", pga);
  if (currentMenuOption == 3)
    displayPrint("Thre %2d%%", thre);
  //if (currentMenuOption == 4) displayPrint("Set %s", menu_setDisp[setDispIndex]);
  if (currentMenuOption == 4)
  {
    if (digitalReadFast(LASER))
    {
      displayPrint("Laser ON");
    }
    else
    {
      displayPrint("LaserOFF");
    }
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 4;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 4)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 4)
    {
      laserTimeout = TIMEOUT_LASER;
      digitalWrite(LASER, !digitalRead(LASER));
    }
    else if (loggedIn)
    {
      currentMenu = MENU_SETUP;
      currentMenuOption = 0;
    }
    else
    {
      currentMenu = MENU_LOGIN;
      currentMenuOption = 0;
    }
  }
}

void showLoginMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (lastKey == BTN_A || lastKey == BTN_B || lastKey == BTN_C || lastKey == BTN_D)
    nextBtn++;

  switch (nextBtn)
  {
  case 0:
    displayPrint("PIN:    ");
    break;
  case 1:
    displayPrint("PIN:*   ");
    passwd = passwd + 1000 * lastKey;
    break;
  case 2:
    displayPrint("PIN:**  ");
    passwd = passwd + 100 * lastKey;
    break;
  case 3:
    displayPrint("PIN:*** ");
    passwd = passwd + 10 * lastKey;
    break;
  case 4:
    displayPrint("PIN:****");
    passwd = passwd + lastKey;
    delay(500);
    if (passwd == 2314)
    {
      currentMenu = MENU_SETUP;
      currentMenuOption = 0;
      displayPrint("PIN  OK!");
      delay(500);
      nextBtn = 0;
      passwd = 0;
      loggedIn = true;
    }
    else
    {
      displayPrint("BAD PIN!");
      delay(500);
      nextBtn = 0;
      passwd = 0;
      loggedIn = false;
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
    }
    break;
  default:
    break;
  }
}

void showSetupMenu(void)
{

  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("Sensor  ");
  if (currentMenuOption == 1)
    displayPrint("Modbus  ");
  if (currentMenuOption == 2)
    displayPrint("Filters ");
  if (currentMenuOption == 3)
    displayPrint("Analog  ");
  if (currentMenuOption == 4)
    displayPrint("Info    ");
  if (currentMenuOption == 5)
  {
    if (digitalReadFast(IR_LED))
    {
      displayPrint("Test  ON");
    }
    else
    {
      displayPrint("Test OFF");
    }
  }

  if (lastKey == BTN_A)
  {
    currentMenu = MENU_MAIN;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 5;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 5)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 0)
    {
      currentMenu = MENU_SENSOR;
      currentMenuOption = 0;
    }
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_MODBUS;
      currentMenuOption = 0;
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_FILTERS;
      currentMenuOption = 0;
    }
    if (currentMenuOption == 3)
    {
      currentMenu = MENU_ANALOG;
      currentMenuOption = 0;
    }
    if (currentMenuOption == 4)
    {
      currentMenu = MENU_INFO;
      currentMenuOption = 0;
    }
    if (currentMenuOption == 5)
    {
      testTimeout = TIMEOUT_TEST;
      intTest = !intTest;
      //currentMenu = MENU_SETUP;
      //currentMenuOption = 3;
    }
  }

  if (lastKey == BTN_AH)
  {
    displayPrint("Logout !");
    passwd = 0;
    nextBtn = 0;
    loggedIn = false;
    delay(500);
    currentMenu = MENU_MAIN;
    currentMenuOption = 0;
  }
}

void showSensorMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("Gain1 %2d", pga1);
  if (currentMenuOption == 1)
    displayPrint("Thre1 %2d", thre1);
  if (currentMenuOption == 2)
    displayPrint("Gain2 %2d", pga2);
  if (currentMenuOption == 3)
    displayPrint("Thre2 %2d", thre2);
  if (currentMenuOption == 4)
    displayPrint("Set %s", menu_setDisp[setDispIndex]);

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { // ESC
    currentMenu = MENU_SETUP;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 4;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 4)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 0)
    {
      currentMenu = MENU_GAIN1;
      currentMenuOption = 0;
      menu_pga = pga1; // local menu variable to avoid changes until saved
    }
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_THRE1;
      currentMenuOption = 0;
      menu_thre = thre1; // local menu variable to avoid changes until saved
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_GAIN2;
      currentMenuOption = 0;
      menu_pga = pga2; // local menu variable to avoid changes until saved
    }
    if (currentMenuOption == 3)
    {
      currentMenu = MENU_THRE2;
      currentMenuOption = 0;
      menu_thre = thre2; // local menu variable to avoid changes until saved
    }
    if (currentMenuOption == 4)
    {
      currentMenu = MENU_SET;
      currentMenuOption = 0;
      menu_set = set; // local menu variable to avoid changes until saved
    }
  }
}

void setGain1Menu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Gain1 %2d", menu_pga);
  else
    displayPrint("      %2d", menu_pga);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_pga > 1)
      menu_pga = menu_pga / 2;
    else
      menu_pga = 64;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (menu_pga < 64)
      menu_pga = menu_pga * 2;
    else
      menu_pga = 1;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SENSOR;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { //SAVE
    pga1 = menu_pga;
    eeprom_writeInt(EE_ADDR_gain_set1, pga1); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_SENSOR;
    currentMenuOption = 0;
  }
}

void setThre1Menu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Thre1 %2d", menu_thre);
  else
    displayPrint("      %2d", menu_thre);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_thre > 20)
      menu_thre = menu_thre - 5;
    else
      menu_thre = 80;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (menu_thre < 80)
      menu_thre = menu_thre + 5;
    else
      menu_thre = 20;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SENSOR;
    currentMenuOption = 1;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    thre1 = menu_thre;
    eeprom_writeInt(EE_ADDR_threshold_set1, thre1); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_SENSOR;
    currentMenuOption = 1;
  }
}

void setGain2Menu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Gain2 %2d", menu_pga);
  else
    displayPrint("      %2d", menu_pga);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_pga > 1)
      menu_pga = menu_pga / 2;
    else
      menu_pga = 64;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (menu_pga < 64)
      menu_pga = menu_pga * 2;
    else
      menu_pga = 1;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SENSOR;
    currentMenuOption = 2;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { //SAVE
    pga2 = menu_pga;
    eeprom_writeInt(EE_ADDR_gain_set2, pga2); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_SENSOR;
    currentMenuOption = 2;
  }
}

void setThre2Menu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Thre2 %2d", menu_thre);
  else
    displayPrint("      %2d", menu_thre);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_thre > 20)
      menu_thre = menu_thre - 5;
    else
      menu_thre = 80;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (menu_thre < 80)
      menu_thre = menu_thre + 5;
    else
      menu_thre = 20;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SENSOR;
    currentMenuOption = 3;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    thre2 = menu_thre;
    eeprom_writeInt(EE_ADDR_threshold_set2, thre2); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_SENSOR;
    currentMenuOption = 3;
  }
}

void setSetMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  switch (menu_set)
  { // display correct text
  case 0:
    setDispIndex = 0; // REL
    break;
  case 1:
    setDispIndex = 3; // MAN1
    break;
  case 2:
    setDispIndex = 4; // MAN2
    break;
  default:
    break;
  }

  if (blinkMenu)
    displayPrint("Set %s", menu_setDisp[setDispIndex]);
  else
    displayPrint("    %s", menu_setDisp[setDispIndex]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_set > 0)
      menu_set--;
    else
      menu_set = 2;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_set < 2)
      menu_set++;
    else
      menu_set = 0;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SENSOR;
    currentMenuOption = 4;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    set = menu_set;
    eeprom_writeInt(EE_ADDR_set, set); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_SENSOR;
    currentMenuOption = 4;
  }
}

void showModbusMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("ID   %3d", modbusID);
  if (currentMenuOption == 1)
    displayPrint("Sp%6d", modbusSpeed);
  if (currentMenuOption == 2)
  {
    for (int i = 0; i < 4; i++)
    { // find actual format in array
      if (modbusFormatArray[i] == modbusFormat)
        actualFormat = i;
    }
    displayPrint("Fmt  %s", menu_modbusFormatDisp[actualFormat]);
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 2;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 2)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { // ESC
    currentMenu = MENU_SETUP;
    currentMenuOption = 1;
  }
  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // ENTER

    if (currentMenuOption == 0)
    {
      currentMenu = MENU_MODBUS_ID;
      menu_modbusID = modbusID;
    } // read actual ID
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_MODBUS_SPEED;
      menu_modbusSpeed = modbusSpeed;
      for (int i = 0; i < 6; i++)
      { // find actual speed in array
        if (modbusSpeedArray[i] * 100 == menu_modbusSpeed)
          actualSpeed = i;
      }
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_MODBUS_FORMAT;
      menu_modbusFormat = modbusFormat;
    }
  }
}

void setModbusID(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("ID   %3d", menu_modbusID);
  else
    displayPrint("     %3d", menu_modbusID);

  if (lastKey == BTN_BH)
  { // hold to increment by 10
    menu_modbusID = menu_modbusID - 10;
  }

  if (lastKey == BTN_CH)
  { // hold to increment by 10
    menu_modbusID = menu_modbusID + 10;
  }

  if (lastKey == BTN_B)
  { // decrement
    menu_modbusID--;
  }
  if (lastKey == BTN_C)
  { // increment
    menu_modbusID++;
  }

  // check valid Modbus Slave ID range 1..247
  if (menu_modbusID < 1)
    menu_modbusID = 247;
  if (menu_modbusID > 247)
    menu_modbusID = 1;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_MODBUS;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    modbusID = menu_modbusID;
    eeprom_writeInt(EE_ADDR_modbus_ID, modbusID); //save to EEPROM

    // restart communication
    Serial1.flush();
    Serial1.end();
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);

    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_MODBUS;
    currentMenuOption = 0;
  }
}

void setModbusSpeed(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Sp%6d", menu_modbusSpeed);
  else
    displayPrint("  %6d", menu_modbusSpeed);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (actualSpeed > 0)
      actualSpeed--;
    else
      actualSpeed = 6;
    menu_modbusSpeed = modbusSpeedArray[actualSpeed] * 100;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (actualSpeed < 6)
      actualSpeed++;
    else
      actualSpeed = 0;
    menu_modbusSpeed = modbusSpeedArray[actualSpeed] * 100;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_MODBUS;
    currentMenuOption = 1;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    modbusSpeed = menu_modbusSpeed;
    eeprom_writeInt(EE_ADDR_modbus_Speed, modbusSpeed / 100); //save to EEPROM

    // restart communication
    Serial1.flush();
    Serial1.end();
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);

    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_MODBUS;
    currentMenuOption = 1;
  }
}

void setModbusFormat(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Fmt  %s", menu_modbusFormatDisp[actualFormat]);
  else
    displayPrint("     %s", menu_modbusFormatDisp[actualFormat]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  { // decrement by 1
    if (actualFormat > 0)
      actualFormat--;
    else
      actualFormat = 3;
    menu_modbusFormat = modbusFormatArray[actualFormat];
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (actualFormat < 3)
      actualFormat++;
    else
      actualFormat = 0;
    menu_modbusFormat = modbusFormatArray[actualFormat];
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_MODBUS;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    modbusFormat = menu_modbusFormat;
    eeprom_writeInt(EE_ADDR_modbus_Format, modbusFormat); //save to EEPROM

    // restart communication
    Serial1.flush();
    Serial1.end();
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);

    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_MODBUS;
    currentMenuOption = 2;
  }
}

void showFiltersMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("fPos%4d", filterPosition);
  if (currentMenuOption == 1)
    displayPrint("fOn %4d", filterOn);
  if (currentMenuOption == 2)
    displayPrint("fOff%4d", filterOff);

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SETUP;
    currentMenuOption = 2;
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 2;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 2)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 0)
    {
      currentMenu = MENU_FILTER_POSITION;
      currentMenuOption = 0;
      menu_filterPosition = filterPosition;
    }
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_FILTER_ON;
      currentMenuOption = 0;
      menu_filterOn = filterOn;
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_FILTER_OFF;
      currentMenuOption = 0;
      menu_filterOff = filterOff;
    }
  }
}

void setFilterPosition(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("fPos%4d", menu_filterPosition);
  else
    displayPrint("    %4d", menu_filterPosition);

  if (lastKey == BTN_BH)
  { // decrement by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterPosition = menu_filterPosition - 10;
    }
    else
    {
      menu_filterPosition = menu_filterPosition - 100;
    }
  }

  if (lastKey == BTN_CH)
  { // increment by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterPosition = menu_filterPosition + 10;
    }
    else
    {
      menu_filterPosition = menu_filterPosition + 100;
    }
  }

  if (lastKey == BTN_B)
  { // decrement
    menu_filterPosition--;
  }
  if (lastKey == BTN_C)
  { // increment
    menu_filterPosition++;
  }

  //check range in ms
  if (menu_filterPosition < 0)
    menu_filterPosition = 9999;
  if (menu_filterPosition > 9999)
    menu_filterPosition = 0;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_FILTERS;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    filterPosition = menu_filterPosition;
    eeprom_writeInt(EE_ADDR_filter_position, filterPosition); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_FILTERS;
    currentMenuOption = 0;
  }
}

void setFilterOn(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("fOn %4d", menu_filterOn);
  else
    displayPrint("    %4d", menu_filterOn);

  if (lastKey == BTN_BH)
  { // decrement by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterOn = menu_filterOn - 10;
    }
    else
    {
      menu_filterOn = menu_filterOn - 100;
    }
  }

  if (lastKey == BTN_CH)
  { // increment by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterOn = menu_filterOn + 10;
    }
    else
    {
      menu_filterOn = menu_filterOn + 100;
    }
  }

  if (lastKey == BTN_B)
  { // decrement
    menu_filterOn--;
  }
  if (lastKey == BTN_C)
  { // increment
    menu_filterOn++;
  }

  //check range in ms
  if (menu_filterOn < 0)
    menu_filterOn = 9999;
  if (menu_filterOn > 9999)
    menu_filterOn = 0;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_FILTERS;
    currentMenuOption = 1;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    filterOn = menu_filterOn;
    eeprom_writeInt(EE_ADDR_filter_on, filterOn); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_FILTERS;
    currentMenuOption = 1;
  }
}

void setFilterOff(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("fOff%4d", menu_filterOff);
  else
    displayPrint("    %4d", menu_filterOff);

  if (lastKey == BTN_BH)
  { // decrement by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterOff = menu_filterOff - 10;
    }
    else
    {
      menu_filterOff = menu_filterOff - 100;
    }
  }

  if (lastKey == BTN_CH)
  { // increment by 1,10,100
    if (btnHoldCounter < 20)
    {
      menu_filterOff = menu_filterOff + 10;
    }
    else
    {
      menu_filterOff = menu_filterOff + 100;
    }
  }

  if (lastKey == BTN_B)
  { // decrement
    menu_filterOff--;
  }

  if (lastKey == BTN_C)
  { // increment
    menu_filterOff++;
  }

  //check range in ms
  if (menu_filterOff < 0)
    menu_filterOff = 9999;
  if (menu_filterOff > 9999)
    menu_filterOff = 0;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_FILTERS;
    currentMenuOption = 2;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    filterOff = menu_filterOff;
    eeprom_writeInt(EE_ADDR_filter_off, filterOff); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_FILTERS;
    currentMenuOption = 2;
  }
}

void showAnalogMenu(void)
{

  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("wBeg%3d%%", windowBegin);
  if (currentMenuOption == 1)
    displayPrint("wEnd%3d%%", windowEnd);
  if (currentMenuOption == 2)
    displayPrint("mPos%s", menu_positionModeDisp[positionMode]);
  if (currentMenuOption == 3)
    displayPrint("AnO %s", menu_analogOutModeDisp[analogOutMode]);
  if (currentMenuOption == 4)
    displayPrint("Offs%4d", positionOffset);

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { // ESC
    currentMenu = MENU_SETUP;
    currentMenuOption = 3;
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 4;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 4)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 0)
    {
      currentMenu = MENU_WINDOW_BEGIN;
      currentMenuOption = 0;
      menu_windowBegin = windowBegin;
    }
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_WINDOW_END;
      currentMenuOption = 0;
      menu_windowEnd = windowEnd;
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_POSITION_MODE;
      currentMenuOption = 0;
      menu_positionMode = positionMode;
    }
    if (currentMenuOption == 3)
    {
      currentMenu = MENU_ANALOG_OUT_MODE;
      currentMenuOption = 0;
      menu_analogOutMode = analogOutMode;
    }
    if (currentMenuOption == 4)
    {
      currentMenu = MENU_POSITION_OFFSET;
      currentMenuOption = 0;
      menu_positionOffset = positionOffset;
    }
  }
}

void setWindowBegin(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("wBeg%3d%%", menu_windowBegin);
  else
    displayPrint("    %3d%%", menu_windowBegin);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  { // decrement
    menu_windowBegin = menu_windowBegin - 5;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment
    menu_windowBegin = menu_windowBegin + 5;
  }

  //check range, min 5, max 45
  if (menu_windowBegin < 5)
    menu_windowBegin = 45;
  if (menu_windowBegin > 45)
    menu_windowBegin = 5;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    windowBegin = menu_windowBegin;
    eeprom_writeInt(EE_ADDR_window_begin, windowBegin); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 0;
  }
}

void setWindowEnd(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("wEnd%3d%%", menu_windowEnd);
  else
    displayPrint("    %3d%%", menu_windowEnd);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  { // decrement
    menu_windowEnd = menu_windowEnd - 5;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment
    menu_windowEnd = menu_windowEnd + 5;
  }

  //check range, min 55, max 95
  if (menu_windowEnd < 55)
    menu_windowEnd = 95;
  if (menu_windowEnd > 95)
    menu_windowEnd = 55;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 1;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    windowEnd = menu_windowEnd;
    eeprom_writeInt(EE_ADDR_window_end, windowEnd); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 1;
  }
}

void setPositionMode(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  // positionMode: HMD = 0, RISE = 1, FALL = 2, PEAK = 3
  if (blinkMenu)
    displayPrint("mPos%s", menu_positionModeDisp[menu_positionMode]);
  else
    displayPrint("    %s", menu_positionModeDisp[menu_positionMode]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_positionMode > 0)
      menu_positionMode--;
    else
      menu_positionMode = 3;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_positionMode < 3)
      menu_positionMode++;
    else
      menu_positionMode = 0;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 2;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    positionMode = menu_positionMode;
    eeprom_writeInt(EE_ADDR_position_mode, positionMode); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 2;
  }
}

void setAnalogOutMode(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  // positionMode: HMD = 0, RISE = 1, FALL = 2, PEAK = 3
  if (blinkMenu)
    displayPrint("AnO %s", menu_analogOutModeDisp[menu_analogOutMode]);
  else
    displayPrint("    %s", menu_analogOutModeDisp[menu_analogOutMode]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_analogOutMode > 0)
      menu_analogOutMode--;
    else
      menu_analogOutMode = 3;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_analogOutMode < 3)
      menu_analogOutMode++;
    else
      menu_analogOutMode = 0;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 3;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    analogOutMode = menu_analogOutMode;
    eeprom_writeInt(EE_ADDR_analog_out_mode, analogOutMode); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 3;
  }
}

void setPositionOffset(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (blinkMenu)
    displayPrint("Offs%4d", menu_positionOffset);
  else
    displayPrint("    %4d", menu_positionOffset);

  if (lastKey == BTN_BH)
  { // decrement
    menu_positionOffset = menu_positionOffset - 10;
  }
  if (lastKey == BTN_CH)
  { // increment
    menu_positionOffset = menu_positionOffset + 10;
  }

  if (lastKey == BTN_B)
  { // decrement
    menu_positionOffset--;
  }
  if (lastKey == BTN_C)
  { // increment
    menu_positionOffset++;
  }

  //check range, min 0, max 2000
  if (menu_positionOffset < 0)
    menu_positionOffset = 2000;
  if (menu_positionOffset > 2000)
    menu_positionOffset = 0;

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 4;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    positionOffset = menu_positionOffset;
    //adc0_busy = 0;
    eeprom_writeInt(EE_ADDR_position_offset, positionOffset); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 4;
  }
}

void showInfoMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
    displayPrint("SHK01-%2d", MODEL_TYPE);
  if (currentMenuOption == 1)
    displayPrint("SN %5d", MODEL_SERIAL_NUMBER);
  if (currentMenuOption == 2)
    displayPrint("FW %5d", FW_VERSION);
  if (currentMenuOption == 3)
    displayPrint("Temp %2dC", celsius);
  if (currentMenuOption == 4)
    displayPrint("MaxT %2dC", max_temperature);
  if (currentMenuOption == 5)
    displayPrint("TT%6d", total_runtime);
  if (currentMenuOption == 6)
    displayPrint("FacReset");

  if (lastKey == BTN_A || lastKey == BTN_AH)
  { //ESC
    currentMenu = MENU_SETUP;
    currentMenuOption = 4;
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 6;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 6)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 6)
    {
      currentMenu = MENU_RESET;
      currentMenuOption = 0;
    }
  }
}

void showResetMenu(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  if (currentMenuOption == 0)
  {
    if (blinkMenu)
    {
      displayPrint("Reset? N");
    }
    else
    {
      displayPrint("       N");
    }
  }
  if (currentMenuOption == 1)
  {
    if (blinkMenu)
    {
      displayPrint("Reset? Y");
    }
    else
    {
      displayPrint("       Y");
    }
  }

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (currentMenuOption > 0)
      currentMenuOption--;
    else
      currentMenuOption = 1;
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  {
    if (currentMenuOption < 1)
      currentMenuOption++;
    else
      currentMenuOption = 0;
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_INFO;
    currentMenuOption = 5;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  {
    if (currentMenuOption == 0)
    {
      currentMenu = MENU_INFO;
      currentMenuOption = 5;
    }
    if (currentMenuOption == 1)
    {
      reset_writeDefaultsToEEPROM();
      config_loadFromEEPROM();

      // restart communication
      Serial1.flush();
      Serial1.end();
      modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);

      displayPrint("RESET!!!");
      delay(500);
      currentMenu = MENU_INFO;
      currentMenuOption = 5;
    }
  }
}

//******************************************************************
// EEPROM

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(unsigned int address, unsigned int value)
{

  EEPROM.write(address, value % 256);     // LSB
  EEPROM.write(address + 1, value / 256); // MSB
}

void eeprom_updateInt(unsigned int address, unsigned int value)
{

  EEPROM.update(address, value % 256);     // LSB
  EEPROM.update(address + 1, value / 256); // MSB
}

// read a unsigned int (two bytes) value from eeprom
unsigned int eeprom_readInt(unsigned int address)
{

  return EEPROM.read(address) + EEPROM.read(address + 1) * 256;
}

void EEPROM_init()
{

  if (
      eeprom_readInt(EE_ADDR_MODEL_TYPE) == MODEL_TYPE &&
      eeprom_readInt(EE_ADDR_MODEL_SERIAL_NUMBER) == MODEL_SERIAL_NUMBER &&
      eeprom_readInt(EE_ADDR_FW_VERSION) == FW_VERSION)
  {

    // loads in ram the eeprom config
    config_loadFromEEPROM();
  }
  else
  {
    config_writeDefaultsToEEPROM();
    config_loadFromEEPROM();
  }
}

void config_loadFromEEPROM()
{
  // loads in ram the eeprom config
  modbusID = eeprom_readInt(EE_ADDR_modbus_ID);
  modbusSpeed = eeprom_readInt(EE_ADDR_modbus_Speed) * 100; // speed/100 to fit 115200 in WORD
  modbusFormat = eeprom_readInt(EE_ADDR_modbus_Format);

  set = eeprom_readInt(EE_ADDR_set);
  pga1 = eeprom_readInt(EE_ADDR_gain_set1);
  thre1 = eeprom_readInt(EE_ADDR_threshold_set1);
  pga2 = eeprom_readInt(EE_ADDR_gain_set2);
  thre2 = eeprom_readInt(EE_ADDR_threshold_set2);

  windowBegin = eeprom_readInt(EE_ADDR_window_begin);
  windowEnd = eeprom_readInt(EE_ADDR_window_end);
  positionMode = eeprom_readInt(EE_ADDR_position_mode);
  analogOutMode = eeprom_readInt(EE_ADDR_analog_out_mode);
  positionOffset = eeprom_readInt(EE_ADDR_position_offset);

  filterPosition = eeprom_readInt(EE_ADDR_filter_position);
  filterOn = eeprom_readInt(EE_ADDR_filter_on);
  filterOff = eeprom_readInt(EE_ADDR_filter_off);

  max_temperature = eeprom_readInt(EE_ADDR_max_temperature);
  total_runtime = eeprom_readInt(EE_ADDR_total_runtime);

  checkSET();
}

void config_writeDefaultsToEEPROM()
{ // flash is empty or program version changed
  // writes sign codes
  eeprom_writeInt(EE_ADDR_MODEL_TYPE, MODEL_TYPE);
  eeprom_writeInt(EE_ADDR_MODEL_SERIAL_NUMBER, MODEL_SERIAL_NUMBER);
  eeprom_writeInt(EE_ADDR_FW_VERSION, FW_VERSION);

  // save defaults to eeprom
  eeprom_writeInt(EE_ADDR_modbus_ID, DEFAULT_MODBUS_ID);
  eeprom_writeInt(EE_ADDR_modbus_Speed, DEFAULT_MODBUS_SPEED / 100); // speed/100 to fit 115200 in WORD
  eeprom_writeInt(EE_ADDR_modbus_Format, DEFAULT_MODBUS_FORMAT);

  eeprom_writeInt(EE_ADDR_set, DEFAULT_SET);
  eeprom_writeInt(EE_ADDR_gain_set1, DEFAULT_GAIN_SET1);
  eeprom_writeInt(EE_ADDR_threshold_set1, DEFAULT_THRESHOLD_SET1);
  eeprom_writeInt(EE_ADDR_gain_set2, DEFAULT_GAIN_SET2);
  eeprom_writeInt(EE_ADDR_threshold_set2, DEFAULT_THRESHOLD_SET2);

  eeprom_writeInt(EE_ADDR_window_begin, DEFAULT_WINDOW_BEGIN);
  eeprom_writeInt(EE_ADDR_window_end, DEFAULT_WINDOW_END);
  eeprom_writeInt(EE_ADDR_position_mode, DEFAULT_POSITION_MODE);
  eeprom_writeInt(EE_ADDR_analog_out_mode, DEFAULT_ANALOG_OUT_MODE);
  eeprom_writeInt(EE_ADDR_position_offset, DEFAULT_POSITION_OFFSET);

  eeprom_writeInt(EE_ADDR_filter_position, DEFAULT_FILTER_POSITION);
  eeprom_writeInt(EE_ADDR_filter_on, DEFAULT_FILTER_ON);
  eeprom_writeInt(EE_ADDR_filter_off, DEFAULT_FILTER_OFF);

  eeprom_writeInt(EE_ADDR_max_temperature, max_temperature);
  eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
}

void reset_writeDefaultsToEEPROM()
{ // reset all values (except calibration values) to fact. defaults
  // writes sign codes
  // eeprom_writeInt(EE_ADDR_MODEL_TYPE, MODEL_TYPE);
  // eeprom_writeInt(EE_ADDR_MODEL_SERIAL_NUMBER, MODEL_SERIAL_NUMBER);
  // eeprom_writeInt(EE_ADDR_FW_VERSION, FW_VERSION);

  // save defaults to eeprom
  eeprom_writeInt(EE_ADDR_modbus_ID, DEFAULT_MODBUS_ID);
  eeprom_writeInt(EE_ADDR_modbus_Speed, DEFAULT_MODBUS_SPEED / 100); // speed/100 to fit 115200 in WORD
  eeprom_writeInt(EE_ADDR_modbus_Format, DEFAULT_MODBUS_FORMAT);

  eeprom_writeInt(EE_ADDR_set, DEFAULT_SET);
  eeprom_writeInt(EE_ADDR_gain_set1, DEFAULT_GAIN_SET1);
  eeprom_writeInt(EE_ADDR_threshold_set1, DEFAULT_THRESHOLD_SET1);
  eeprom_writeInt(EE_ADDR_gain_set2, DEFAULT_GAIN_SET2);
  eeprom_writeInt(EE_ADDR_threshold_set2, DEFAULT_THRESHOLD_SET2);

  eeprom_writeInt(EE_ADDR_window_begin, DEFAULT_WINDOW_BEGIN);
  eeprom_writeInt(EE_ADDR_window_end, DEFAULT_WINDOW_END);
  eeprom_writeInt(EE_ADDR_position_mode, DEFAULT_POSITION_MODE);
  eeprom_writeInt(EE_ADDR_analog_out_mode, DEFAULT_ANALOG_OUT_MODE);
  // eeprom_writeInt(EE_ADDR_position_offset, DEFAULT_POSITION_OFFSET);

  eeprom_writeInt(EE_ADDR_filter_position, DEFAULT_FILTER_POSITION);
  eeprom_writeInt(EE_ADDR_filter_on, DEFAULT_FILTER_ON);
  eeprom_writeInt(EE_ADDR_filter_off, DEFAULT_FILTER_OFF);

  // eeprom_writeInt(EE_ADDR_max_temperature, max_temperature);
  // eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
}

// check SET and load proper settings
void checkSET()
{
  switch (set)
  {
  case 0:
    if (digitalRead(SET_IN))
    {
      pga = pga1;
      thre = thre1;
      setDispIndex = 1;
    }
    else
    {
      pga = pga2;
      thre = thre2;
      setDispIndex = 2;
    }
    break;
  case 1:
    pga = pga1;
    thre = thre1;
    setDispIndex = 3;
    break;
  case 2:
    pga = pga2;
    thre = thre2;
    setDispIndex = 4;
    break;
  default:
    break;
  }
  thre256 = thre * 256 / 100 - 1;
}

void checkTEST()
{
  if (!digitalRead(TEST_IN))
    extTest = true;
  else
    extTest = false;

  if (extTest || intTest)
  {
    digitalWrite(IR_LED, HIGH);
    if (blinkMenu)
      digitalWriteFast(LED_ALARM, HIGH);
    else
      digitalWriteFast(LED_ALARM, LOW);
  }
  else
    digitalWrite(IR_LED, LOW);
}

void checkALARM()
{
  //check internal temperature
  temp = adc->adc1->readSingle();
  //celsius = (181964504 - 69971 * temp12) >> 12 ; //Vref 1.2
  celsius = 25.0 + 0.46977 * (892.43 - temp); //Vref 3.3
  if (celsius > max_temperature)
  {
    max_temperature = celsius;
    eeprom_writeInt(EE_ADDR_max_temperature, max_temperature);
  }

  // check runtime
  if (!hourTimeout)
  { // every hour
    hourTimeout = 3600000;
    total_runtime++;
    if ((total_runtime % 4) == 1)
    { // every 4 hours
      eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
    }
  }

  //check alarms
  if ((motorTimeDiff > (6000 + 50)) || (motorTimeDiff < (6000 - 50)))
  { //motor alarm if not 6000us per rot.
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); //negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 0;
    }
  }                                                                                                       //set ALARM
  else if ((celsius > 55) || ((celsius > 50) && (currentMenu == MENU_ALARM) && (currentMenuOption == 1))) // 5 deg hysteresis internal temperature alarm
  {                                                                                                       // temp alarm
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); //negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 1;
    }
  }
  else if (extTest)
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); //NO ALARM => negative output: 24V=OK , but keep LED_ALARM blinking
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 2;
    }
  }
  else if (intTest && (currentMenu != MENU_SETUP))
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); //NO ALARM => negative output: 24V=OK, but keep LED_ALARM blinking
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 3;
    }
  }
  else
  {                                        // no alarm, no warnings
    digitalWriteFast(LED_ALARM, LOW);      //no ALARM
    digitalWriteFast(OUT_ALARM_NEG, HIGH); //negative output: 24V=OK
    alarmChecked = false;
    if (currentMenu == MENU_ALARM)
    {
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
    }
  }
}

//*****************************************************************
// SPI send 2 x 16 bit value
void updateSPI(int valueAN1, int valueAN2)
{
  // gain control of the SPI port
  // and configure settings
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // max 3.3MBPS, CPOL=0, CPHA=0
  digitalWrite(SPI_CS, LOW);                                       // take LATCH pin LOW
  SPI.transfer16(valueAN2);
  SPI.transfer16(valueAN1);
  digitalWrite(SPI_CS, HIGH); // update DAC registers on AD420
  // release control of the SPI port
  SPI.endTransaction();
}

// INTERRUPT ROUTINES

// button interrupts
//*****************************************************************
void checkButtonA()
{
  if (digitalReadFast(PIN_BTN_A))
  {
    if (BtnPressedATimeout || resultButtonA == STATE_LONG)
      BtnReleasedA = true;
  }
  else
  {
    BtnReleasedA = false;
    BtnPressedATimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonB()
{
  if (digitalReadFast(PIN_BTN_B))
  {
    if (BtnPressedBTimeout || resultButtonB == STATE_LONG)
      BtnReleasedB = true;
  }
  else
  {
    BtnReleasedB = false;
    BtnPressedBTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonC()
{
  if (digitalReadFast(PIN_BTN_C))
  {
    if (BtnPressedCTimeout || resultButtonC == STATE_LONG)
      BtnReleasedC = true;
  }
  else
  {
    BtnReleasedC = false;
    BtnPressedCTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonD()
{
  if (digitalReadFast(PIN_BTN_D))
  {
    if (BtnPressedDTimeout || resultButtonD == STATE_LONG)
      BtnReleasedD = true;
  }
  else
  {
    BtnReleasedD = false;
    BtnPressedDTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
// Timer interrupts
void timer500us_isr(void)
{ //every 500us
  //motor pulse
  digitalWriteFast(MOTOR_CLK, !digitalReadFast(MOTOR_CLK));

  //update timeouts

  if (refreshMenuTimeout)
  {
    refreshMenuTimeout--;
  }

  if (laserTimeout)
  {
    laserTimeout--;
  }
  else
  {
    digitalWrite(LASER, LOW);
  }

  if (testTimeout)
  {
    testTimeout--;
  }
  else
  {
    intTest = false;
  }

  if (menuTimeout)
  {
    menuTimeout--;
    if (!menuTimeout)
    {
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
      alarmChecked = false;
      loggedIn = false;
    }
  }

  //buttons timeouts
  if (BtnPressedATimeout)
  {
    BtnPressedATimeout--;
    if (!BtnReleasedA && (BtnPressedATimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonA = STATE_LONG;
      BtnPressedATimeout = 0;
    }
    if (BtnReleasedA && (BtnPressedATimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonA = STATE_SHORT;
      BtnPressedATimeout = 0;
    }
  }

  if (BtnPressedBTimeout)
  {
    BtnPressedBTimeout--;
    if (!BtnReleasedB && (BtnPressedBTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonB = STATE_LONG;
      BtnPressedBTimeout = 0;
    }
    if (BtnReleasedB && (BtnPressedBTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonB = STATE_SHORT;
      BtnPressedBTimeout = 0;
    }
  }

  if (BtnPressedCTimeout)
  {
    BtnPressedCTimeout--;
    if (!BtnReleasedC && (BtnPressedCTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonC = STATE_LONG;
      BtnPressedCTimeout = 0;
    }
    if (BtnReleasedC && (BtnPressedCTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonC = STATE_SHORT;
      BtnPressedCTimeout = 0;
    }
  }

  if (BtnPressedDTimeout)
  {
    BtnPressedDTimeout--;
    if (!BtnReleasedD && (BtnPressedDTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonD = STATE_LONG;
      BtnPressedDTimeout = 0;
    }
    if (BtnReleasedD && (BtnPressedDTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonD = STATE_SHORT;
      BtnPressedDTimeout = 0;
    }
  }

  if (digitalReadFast(MOTOR_CLK))
  {
    hourTimeout--;        // every 1ms
    pulsetime = micros(); // for position compensation
  }
}

// motor (from HALL sensor) interrupt
void motor_isr(void)
{
  motorPulseIndex++;

  if (motorPulseIndex > 5)
  { // one time per turn
    motorTimeOld = motorTimeNow;
    motorTimeNow = micros();
    motorTimeDiff = motorTimeNow - motorTimeOld; // time of one rotation = 6000us
    motorPulseIndex = 0;
  } // one time per turn

  if ((motorTimeDiff < (6000 + 50)) && (motorTimeDiff > (6000 - 50))) //motor is at full speed 6000us per rot, no motor alarm.
  {
    holdingRegs[EXEC_TIME_TRIGGER] = micros() - pulsetime;
    delayOffset = (positionOffset % 1000 - holdingRegs[EXEC_TIME_TRIGGER]) % 1000; // compensation for HALL magnets position
    if (delayOffset < 0)                                                           // rotate delayOffset between 0 - 1000
      delayOffset = 1000 + delayOffset;
    TeensyDelay::trigger(delayOffset, 0);
  }
}

void callback_delay()
{
  if (!adc0_busy) // previous ADC conversion ended
  {
    exectime = micros();
    memset((void *)adc0_buf, 0, sizeof(adc0_buf)); // clear DMA buffer

    //adc0_busy = 1;

    // update PGA
    adc->adc0->enablePGA(pga);
    adc->analogReadDifferential(A10, A11, ADC_0); //start ADC_0 differential

    //adc0_dma.enable();
    adc->adc0->enableDMA();
    adc0_dma.enable();

    NVIC_DISABLE_IRQ(IRQ_PDB);
    //Serial.println("Start PDB");
    adc->adc0->startPDB(freq); //check ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);

    updateResults(); // update outputs from adc_data[] during next ADC conversion
    holdingRegs[EXEC_TIME] = micros() - exectime;
  }
  else
  {
    holdingRegs[EXEC_TIME] = motorPulseIndex; // just for debugging
    //adc0_dma_isr();
  }
  adc0_busy = true;
}

void adc0_dma_isr(void)
{
  adc0_dma.clearInterrupt();
  adc0_dma.clearComplete();
  //Serial.println("DMA interrupt");
  //PDB0_CH1C1 = 0; // clear PDB channel control register - should be implemented in stopPDB()
  PDB0_CH0C1 = 0;

  adc->adc0->stopPDB();
  adc0_dma.disable();
  adc->adc0->disableDMA();
  adc0_dma.disable();

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++) // copy DMA buffer
  {
    if (adc0_buf[i] < 0)
      adc_data[i] = 0;
    else
      adc_data[i] = adc0_buf[i];
  }

  adc0_busy = false;
  holdingRegs[EXEC_TIME_ADC] = micros() - exectime; // exectime of adc conversions
}

void updateResults()
{
  int hmdThreshold = 0;
  int winBegin = 0;
  int winEnd = 0;
  int peakValue = 0;
  int peak[ANALOG_BUFFER_SIZE] = {0};
  long risingEdgeTime = 0;
  long fallingEdgeTime = 0;
  long peakValueTime = 0;
  int positionValue = 0;
  int positionValueAvg = 0;

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++)
  {
    // calculate thresholds (with hysteresis) first for speed up calculation
    if (!digitalReadFast(FILTER_PIN))
    {
      hmdThreshold = thre256 + hmdThresholdHyst;
      winBegin = windowBegin * 2;
      winEnd = windowEnd * 2;
    }
    else
    {
      hmdThreshold = thre256 - hmdThresholdHyst;
      winBegin = windowBegin * 2 - 5;
      winEnd = windowEnd * 2 + 5;
    }

    if (i == winBegin)
      peak[i] = adc_data[i]; //check first peak

    if ((i > winBegin) && (i < winEnd)) // if value is inside the measuring window
    {
      // check peak
      if (adc_data[i] > peakValue)
      {
        peakValue = adc_data[i];
      }
      peak[i] = peakValue;

      // check threshold crossing
      if (peakValue > hmdThreshold) // check threshold crossing with hysteresis
      {
        // HMD mode
        if ((positionMode == 0) && !peakValueTime)
        {
          peakValueTime = i * 5;
          digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
        }

        // RISING EDGE mode
        if ((positionMode == 1) && (!risingEdgeTime)) // only first occurence
        {
          if (peak[i - 1] <= hmdThreshold)
          {
            risingEdgeTime = i * 5;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }

        // check for falling edge
        if (positionMode == 2) // only the first occurence
        {
          if ((adc_data[i] < thre256 - hmdThresholdHyst) && (!fallingEdgeTime)) // added additional hysteresis to avoid flickering
          {
            fallingEdgeTime = i * 5;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }

        // check for peak (but signal can be unstable)
        if (positionMode == 3)
        {
          if (peak[i - 1] + 5 < peakValue) // check for peak
          {
            peakValueTime = i * 5;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }
      }
    }
  }

  // check SIGNAL PRESENT
  if ((peakValue < thre256 - hmdThresholdHyst) || (!peakValueTime && !risingEdgeTime && !fallingEdgeTime))
  {
    digitalWriteFast(FILTER_PIN, LOW);
  }

  if (extTest || intTest)
  {
    digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
  }

  // update SIGNAL PRESENT bounce2 filter
  filterOnOff.update();

  if (filterOnOff.rose())
  {
    filterOnOff.interval(filterOff); // update filter interval
    digitalWriteFast(LED_SIGNAL, HIGH);
    digitalWriteFast(OUT_SIGNAL_NEG, LOW);
  }

  if (filterOnOff.fell())
  {
    filterOnOff.interval(filterOn); //update filter interval
    digitalWriteFast(LED_SIGNAL, LOW);
    digitalWriteFast(OUT_SIGNAL_NEG, HIGH);
  }

  if (digitalReadFast(LED_SIGNAL)) // update position only when SIGNAL PRESENT
  {
    switch (positionMode)
    { // for display
    case 1:
      positionValueDisp = risingEdgeTime;
      break;
    case 2:
      positionValueDisp = fallingEdgeTime;
      break;
    case 3:
    case 0:
      positionValueDisp = peakValueTime;
      break;
    }
  }
  else
    positionValueDisp = 0;

  positionValueAvg = approxSimpleMovingAverage(positionValueDisp, filterPosition);

  // remap and send to SPI
  positionValue = constrain(positionValueAvg, windowBegin * 10, windowEnd * 10); // only within measuring window

  positionValueAvgDisp = map(positionValue, windowBegin * 10, windowEnd * 10, 0, 1000); // for display range 0 - 1000

  positionValue = map(positionValue, windowBegin * 10, windowEnd * 10, 0, 65535); // remap for DAC range

  peakValueDisp = map(peakValue, 0, 255, 0, 100); // for display 0 - 100%

  peakValue = map(peakValue, 0, 255, 0, 65535); // remap for DAC range

  if (extTest || intTest) // check test mode and set outputs to 50% and 12mA
  {
    positionValueDisp = 500;    // 50% of display range 0 - 1000
    positionValueAvgDisp = 500; // 50% of display range 0 - 1000
    positionValue = 0x7FFF;     // 12mA on position analog output
    peakValueDisp = 75;         // 75% of display range 0 - 100%
    peakValue = 0xBFFF;         // 16mA on intensity analog output
  }

  switch (analogOutMode)
  { //an1/an2: "1Int2Pos" = 0, "1Pos2Int" = 1, "1Int2Int" = 2, "1Pos2Pos" = 3
  case 0:
    updateSPI(peakValue, positionValue); // range is 2x 16bit
    break;
  case 1:
    updateSPI(positionValue, peakValue);
    break;
  case 2:
    updateSPI(peakValue, peakValue);
    break;
  case 3:
    updateSPI(positionValue, positionValue);
    break;
  }

  //if (dataSent && motorPulseIndex == 0) // prepare data for visualization on PC, only first mirror
  if (dataSent && motorPulseIndex == (filterPosition % 6)) // possibility to view different mirrors by changing positionFilter
  {
    for (byte i = 0; i < (MOTOR_TIME_DIFF - AN_VALUES); i++) // MOTOR_TIME_DIFF = AN_VALUES + 25
    {
      value_buffer[i] = adc_data[i * 8 + 4] << 8 | adc_data[i * 8]; // MSB = value_buffer[i*8+4] , LSB = value_buffer[i*8] ; only 50 of 200
    }

    dataSent = false;
  }
}

// exponential moving average
long approxSimpleMovingAverage(int new_value, int period)
{

  if (filterPosition)
  {                                   // avoid div/0
    if (!digitalReadFast(LED_SIGNAL)) // clear values
    {
      sma = 0;
    }
    else
    {
      sma *= (period - 1);
      sma += new_value;
      sma /= period;
    }
    return sma;
  }
  else
    return new_value;
}

void checkSTATUS()
{
  bitWrite(io_state, IO_LASER, digitalRead(LASER));
  bitWrite(io_state, IO_IR_LED, digitalRead(IR_LED));
  bitWrite(io_state, IO_TEST_IN, !digitalRead(TEST_IN));
  bitWrite(io_state, IO_SET_IN, !digitalRead(SET_IN));
  bitWrite(io_state, IO_ALARM_OUT, !digitalRead(OUT_ALARM_NEG));
  bitWrite(io_state, IO_SIGNAL_OUT, digitalRead(LED_SIGNAL));
  bitWrite(io_state, IO_LED_ALARM, digitalRead(LED_ALARM));
  bitWrite(io_state, IO_LED_SIGNAL, digitalRead(LED_SIGNAL));
  bitWrite(io_state, IO_LED_POWER, digitalRead(LED_POWER));
  bitWrite(io_state, IO_BTN_A, !digitalRead(PIN_BTN_A));
  bitWrite(io_state, IO_BTN_B, !digitalRead(PIN_BTN_B));
  bitWrite(io_state, IO_BTN_C, !digitalRead(PIN_BTN_C));
  bitWrite(io_state, IO_BTN_D, !digitalRead(PIN_BTN_D));
}

void checkModbus()
{

  holdingRegs[MODBUS_ID] = modbusID;
  holdingRegs[MODBUS_SPEED] = modbusSpeed / 100; // baud rate/100 to fit into word
  holdingRegs[MODBUS_FORMAT] = modbusFormat;

  holdingRegs[SET] = set;
  holdingRegs[GAIN_SET1] = pga1;
  holdingRegs[THRESHOLD_SET1] = thre1;
  holdingRegs[GAIN_SET2] = pga2;
  holdingRegs[THRESHOLD_SET2] = thre2;

  holdingRegs[WINDOW_BEGIN] = windowBegin;
  holdingRegs[WINDOW_END] = windowEnd;
  holdingRegs[POSITION_MODE] = positionMode;
  holdingRegs[ANALOG_OUT_MODE] = analogOutMode;
  holdingRegs[POSITION_OFFSET] = positionOffset;

  holdingRegs[FILTER_POSITION] = filterPosition;
  holdingRegs[FILTER_ON] = filterOn;
  holdingRegs[FILTER_OFF] = filterOff;

  holdingRegs[ACT_TEMPERATURE] = celsius;
  holdingRegs[MAX_TEMPERATURE] = max_temperature;
  holdingRegs[TOTAL_RUNTIME] = total_runtime;
  holdingRegs[IO_STATE] = io_state;

  holdingRegs[MOTOR_TIME_DIFF] = motorTimeDiff;
  holdingRegs[OFFSET_DELAY] = delayOffset;

  // updated in updateResults()
  holdingRegs[PEAK_VALUE] = peakValueDisp;
  holdingRegs[POSITION_VALUE] = positionValueDisp;
  holdingRegs[POSITION_VALUE_AVG] = positionValueAvgDisp;

  if (!dataSent)
  {
    for (byte i = 0; i < (MOTOR_TIME_DIFF - AN_VALUES); i++) // MOTOR_TIME_DIFF = AN_VALUES + 25
    {
      holdingRegs[i + AN_VALUES] = value_buffer[i]; // values stored properly in updateResults() to save memory
    }

    dataSent = true;
  }

  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);

  // check changes made via ModBus - if values are valid, save them in EEPROM

  if ((holdingRegs[MODBUS_ID] != modbusID) || (holdingRegs[MODBUS_SPEED] * 100 != modbusSpeed) || (holdingRegs[MODBUS_FORMAT] != modbusFormat))
  {
    if (holdingRegs[MODBUS_ID] != modbusID && holdingRegs[MODBUS_ID] > 0 && holdingRegs[MODBUS_ID] < 248)
    {
      modbusID = holdingRegs[MODBUS_ID];
      eeprom_writeInt(EE_ADDR_modbus_ID, modbusID);
    }

    if (holdingRegs[MODBUS_SPEED] * 100 != modbusSpeed)
    {
      switch (holdingRegs[MODBUS_SPEED] * 100)
      {
      case 300:
      case 600:
      case 1200:
      case 2400:
      case 4800:
      case 9600:
      case 14400:
      case 19200:
      case 28800:
      case 38400:
      case 57600:
      case 115200:
        modbusSpeed = holdingRegs[MODBUS_SPEED] * 100;
        eeprom_writeInt(EE_ADDR_modbus_Speed, modbusSpeed / 100);
        break;
      default:
        break;
      }
    }

    if (holdingRegs[MODBUS_FORMAT] != modbusFormat)
    {
      switch (holdingRegs[MODBUS_FORMAT])
      {
      case SERIAL_8N1:
      case SERIAL_8E1:
      case SERIAL_8O1:
      case SERIAL_8N2:
        modbusFormat = holdingRegs[MODBUS_FORMAT];
        eeprom_writeInt(EE_ADDR_modbus_Format, modbusFormat);
        break;
      default:
        break;
      }
    }

    // restart communication
    Serial1.flush();
    Serial1.end();
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);
  }

  if (holdingRegs[SET] != set && holdingRegs[SET] < 3)
  { // RELAY = 0 (REL1 || REL2), MAN1 = 1, MAN2 = 2
    set = holdingRegs[SET];
    eeprom_writeInt(EE_ADDR_set, set);
  }

  if (holdingRegs[GAIN_SET1] != pga1)
  {
    switch (holdingRegs[GAIN_SET1])
    { // check for valid values
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
      pga1 = holdingRegs[GAIN_SET1];
      eeprom_writeInt(EE_ADDR_gain_set1, pga1);
      break;
    default:
      break;
    }
  }

  if (holdingRegs[THRESHOLD_SET1] != thre1 && holdingRegs[THRESHOLD_SET1] >= 20 && holdingRegs[THRESHOLD_SET1] <= 80)
  {
    thre1 = holdingRegs[THRESHOLD_SET1];
    eeprom_writeInt(EE_ADDR_threshold_set1, thre1);
  }

  if (holdingRegs[GAIN_SET2] != pga2)
  {
    switch (holdingRegs[GAIN_SET2])
    {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
      pga2 = holdingRegs[GAIN_SET2];
      eeprom_writeInt(EE_ADDR_gain_set2, pga2);
      break;
    default:
      break;
    }
  }

  if (holdingRegs[THRESHOLD_SET2] != thre2 && holdingRegs[THRESHOLD_SET2] >= 20 && holdingRegs[THRESHOLD_SET2] <= 80)
  {
    thre2 = holdingRegs[THRESHOLD_SET2];
    eeprom_writeInt(EE_ADDR_threshold_set2, thre2);
  }

  if (holdingRegs[WINDOW_BEGIN] != windowBegin && holdingRegs[WINDOW_BEGIN] >= 5 && holdingRegs[WINDOW_BEGIN] <= 45)
  {
    windowBegin = holdingRegs[WINDOW_BEGIN];
    eeprom_writeInt(EE_ADDR_window_begin, windowBegin);
  }

  if (holdingRegs[WINDOW_END] != windowEnd && holdingRegs[WINDOW_END] >= 55 && holdingRegs[WINDOW_END] <= 95)
  {
    windowEnd = holdingRegs[WINDOW_END];
    eeprom_writeInt(EE_ADDR_window_end, windowEnd);
  }

  if (holdingRegs[POSITION_MODE] != positionMode && holdingRegs[POSITION_MODE] < 4)
  {
    positionMode = holdingRegs[POSITION_MODE];
    eeprom_writeInt(EE_ADDR_position_mode, positionMode);
  }

  if (holdingRegs[ANALOG_OUT_MODE] != analogOutMode && holdingRegs[ANALOG_OUT_MODE] < 4)
  {
    analogOutMode = holdingRegs[ANALOG_OUT_MODE];
    eeprom_writeInt(EE_ADDR_analog_out_mode, analogOutMode);
  }

  if (holdingRegs[POSITION_OFFSET] != positionOffset && holdingRegs[POSITION_OFFSET] >= 0 && holdingRegs[POSITION_OFFSET] <= 2000)
  {
    positionOffset = holdingRegs[POSITION_OFFSET];
    eeprom_writeInt(EE_ADDR_position_offset, positionOffset);
  }

  if (holdingRegs[FILTER_POSITION] != filterPosition && holdingRegs[FILTER_POSITION] < 10000)
  {
    filterPosition = holdingRegs[FILTER_POSITION];
    eeprom_writeInt(EE_ADDR_filter_position, filterPosition);
  }

  if (holdingRegs[FILTER_ON] != filterOn && holdingRegs[FILTER_ON] < 10000)
  {
    filterOn = holdingRegs[FILTER_ON];
    eeprom_writeInt(EE_ADDR_filter_on, filterOn);
  }

  if (holdingRegs[FILTER_OFF] != filterOff && holdingRegs[FILTER_OFF] < 10000)
  {
    filterOff = holdingRegs[FILTER_OFF];
    eeprom_writeInt(EE_ADDR_filter_off, filterOff);
  }

  if (holdingRegs[IO_STATE] != io_state)
  {
    if (holdingRegs[IO_STATE] & (1 << IO_LASER))
    { // check if IO_LASER bit is set
      laserTimeout = TIMEOUT_LASER;
      digitalWrite(LASER, HIGH);
    }
    else
    {
      digitalWrite(LASER, LOW);
      laserTimeout = 0;
    }

    if (holdingRegs[IO_STATE] & (1 << IO_IR_LED))
    { // check if IO_IR_LED bit is set
      digitalWrite(IR_LED, HIGH);
      testTimeout = TIMEOUT_TEST;
      intTest = true;
    }
    else
    {
      digitalWrite(IR_LED, LOW);
      intTest = false;
    }
  }
}

//check void ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);

// pdb interrupt is enabled in case you need it.
// void pdb_isr(void) {
//         PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt
//        // NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt
//         //digitalWriteFast(LED_BUILTIN,!digitalReadFast(LED_BUILTIN));
//         Serial.println("PDB interrupt");
// }