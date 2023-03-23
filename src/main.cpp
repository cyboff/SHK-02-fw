// firmware for SHK-03 board

#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

// for ADC
#include <ADC.h>
#include <DMAChannel.h>

// #include "RingBufferDMA.h"

// for SPI
#include <SPI.h>

// for filters
#include <Bounce2.h>

// for ModBus
#include "SimpleModbusSlave.h"

#include "Variables.h"
#include "EEPROMfunctions.h"
#include "DisplayMenus.h"

uint16_t holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

// Keyboard times
#define BTN_DEBOUNCE_TIME 200 // debounce time (*500us) to prevent flickering when pressing or releasing the button
#define BTN_HOLD_TIME 3000    // holding period (*500us) how long to wait for press+hold event

// create am instance of the LED display library:
LedDisplay myDisplay = LedDisplay(LED_DATA_PIN, LED_RS_PIN, LED_CLK_PIN, LED_EN_PIN, LED_RST_PIN, DISPLAY_LENGHT);
int brightness = 15; // screen brightness

// filters
Bounce filterOnOff = Bounce();
float sma = 0;

// configure ADC
#define ADC0_AVERAGING 1
#define ANALOG_BUFFER_SIZE 200

volatile DMAMEM int16_t adc0_buf[ANALOG_BUFFER_SIZE] = {}; // buffer 1...

ADC *adc = new ADC(); // adc object
DMAChannel adc0_dma;
// References for ISRs...
// extern void adc0_dma_isr(void);

volatile uint8_t adc_data[ANALOG_BUFFER_SIZE] = {}; // ADC_0 9-bit resolution for differential - sign + 8 bit
volatile uint16_t value_buffer[25] = {};
// volatile int value_peak[ANALOG_BUFFER_SIZE];
volatile boolean adc0_busy = false;
unsigned int freq = 400000;         // PDB frequency
volatile int adc0Value = 0;         // analog value
volatile int analogBufferIndex = 0; // analog buffer pointer
volatile int delayOffset = 0;
// sensor variables

int hmdThresholdHyst = 13;

// Timers
IntervalTimer timer500us; // timer for motor speed and various timeouts
IntervalTimer timerDelay; // timer for precise offset

volatile int motorPulseIndex = 0;
volatile long motorTimeOld = 0;
volatile long motorTimeNow = 0;
volatile int motorTimeDiff = 0;
volatile int peakValue = 0;
volatile int positionValue = 0;

// Buttons
volatile int BtnPressedATimeout = 0;
volatile int BtnPressedBTimeout = 0;
volatile int BtnPressedCTimeout = 0;
volatile int BtnPressedDTimeout = 0;
volatile boolean BtnReleasedA = true;
volatile boolean BtnReleasedB = true;
volatile boolean BtnReleasedC = true;
volatile boolean BtnReleasedD = true;

// check and load proper settings
void InitVariables();
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

void callback_delay_isr();
void adc0_dma_isr(void);
void updateResults();

// exponential moving average
long approxSimpleMovingAverage(int new_value, int period);

void checkSTATUS();
void checkModbus();

void setup()
{
  InitVariables();
  // initialize LEDs and I/O

  // pinMode(LED_BUILTIN, OUTPUT); //conflicts with SPI_CLK!!!
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
  checkSET();

  // initialize filters
  pinMode(FILTER_PIN, OUTPUT);
  digitalWriteFast(FILTER_PIN, LOW);
  filterOnOff.attach(FILTER_PIN);
  filterOnOff.interval(filterOn);

  // initialize SPI

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

  // Serial.begin(modbusSpeed);

  // modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);
  modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, 1100, 0); // for compatibility with old SDIS

  // initialize ADC

  ///// ADC0 in differential mode with PGA for signal from photodiode ////

  pinMode(A10, INPUT_DISABLE); // analog input P differential for PGA
  pinMode(A11, INPUT_DISABLE); // analog input N differential for PGA

  adc->adc0->setAveraging(ADC0_AVERAGING); // set number of averages
  adc->adc0->setResolution(9);             // set bits of resolution - 9 bit for differential

  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  // adc->adc0->setReference(ADC_REFERENCE::REF_1V2); // use default 3.3V for input signal > 1.2V

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
  // adc->analogReadDifferential(A10, A11, ADC_0);             //start ADC_0 differential
  adc->analogRead(ADC_INTERNAL_SOURCE::TEMP_SENSOR, ADC_1); // start ADC_1 single, temp sensor internal pin 38

  // adc->enableInterrupts(ADC_0);
  // adc->startContinuousDifferential(A10, A11, ADC_0);
  adc->startContinuous(38, ADC_1); // do not want to accept ADC_INTERNAL_SOURCE::TEMP_SENSOR

  // motor configuration and startup
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW); // motor enable
  pinMode(MOTOR_CLK, OUTPUT);      // motor output pulses

  pinMode(MOTOR_ALARM, INPUT_PULLUP); // motor input pulses
  NVIC_SET_PRIORITY(IRQ_PORTD, 16);   // pin 14 - PortD, see schematic of Teensy 3.2
  if (positionOffset < 1000)          // depends on motor HALL sensors & mirror position - choose the best to have no timing issues
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, FALLING);
  else
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, RISING);

  // NVIC_SET_PRIORITY(IRQ_PORTC, 0);

  // motor slow start
  timer500us.priority(0);
  timerDelay.priority(1);

  for (int speed = 20; speed <= 100; speed++)
  {
    timer500us.end();
    timer500us.begin(timer500us_isr, 50000 / speed); // motor output pulses slowly going to 500us
    displayPrint("Mot=%3d%%", speed);
    delay(100);
  }

  // clear data buffers
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

  // show info on LED display
  displayMenu();
}

//******************************************************************
// check SET and load proper settings
void checkSET()
{
  switch (set) // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
  {
  case 1:
    pga = pga1;
    thre = thre1;
    setDispIndex = 0; // MAN1
    break;
  case 2:
    pga = pga2;
    thre = thre2;
    setDispIndex = 1; // MAN2
    break;
  case 3: // REL
    if (digitalRead(SET_IN))
    {
      pga = pga1;
      thre = thre1;
      setDispIndex = 3; // REL1
    }
    else
    {
      pga = pga2;
      thre = thre2;
      setDispIndex = 4; // REL2
    }
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
  // check internal temperature
  temp = adc->adc1->readSingle();
  // celsius = (181964504 - 69971 * temp12) >> 12 ; //Vref 1.2
  celsius = 25.0 + 0.46977 * (892.43 - temp); // Vref 3.3
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
    {                                         // every 4 hours
      total_runtime = total_runtime & 0xFFFF; // prevent overload
      eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
    }
  }

  // check alarms
  if ((motorTimeDiff > (6000 + 50)) || (motorTimeDiff < (6000 - 50)))
  { // motor alarm if not 6000us per rot.
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); // negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 0;
    }
  }                                                                                                       // set ALARM
  else if ((celsius > 60) || ((celsius > 55) && (currentMenu == MENU_ALARM) && (currentMenuOption == 1))) // 5 deg hysteresis internal temperature alarm
  {                                                                                                       // temp alarm
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); // negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 1;
    }
  }
  else if (extTest)
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // NO ALARM => negative output: 24V=OK , but keep LED_ALARM blinking
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 2;
    }
  }
  else if (intTest && (currentMenu != MENU_SETUP))
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // NO ALARM => negative output: 24V=OK, but keep LED_ALARM blinking
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 3;
    }
  }
  else
  {                                        // no alarm, no warnings
    digitalWriteFast(LED_ALARM, LOW);      // no ALARM
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // negative output: 24V=OK
    alarmChecked = false;
    if (currentMenu == MENU_ALARM)
    {
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
    }
  }
}

//****************************************************************

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
{ // every 500us
  // motor pulse
  digitalToggleFast(MOTOR_CLK);
  //digitalWriteFast(MOTOR_CLK, !digitalReadFast(MOTOR_CLK));

  // update timeouts

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
    digitalWriteFast(LASER, LOW);
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

  // buttons timeouts
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

  if ((motorTimeDiff < (6000 + 50)) && (motorTimeDiff > (6000 - 50))) // motor is at full speed 6000us per rot, no motor alarm.
  {
    holdingRegs[EXEC_TIME_TRIGGER] = micros() - pulsetime;
    delayOffset = (positionOffset % 1000 - holdingRegs[EXEC_TIME_TRIGGER]) % 1000; // compensation for HALL magnets position
    if (delayOffset < 0)                                                           // rotate delayOffset between 0 - 1000
      delayOffset = 1000 + delayOffset;

    timerDelay.begin(callback_delay_isr, delayOffset);
  }
}

void callback_delay_isr()
{
  timerDelay.end();
  if (!adc0_busy) // previous ADC conversion ended
  {
    exectime = micros();

    // warning! SPI makes noise, do not send data through SPI when ADC is running
    // updating SPI values from previous ADC sequence now, before new ADC sequence starts

    switch (analogOutMode)
    { // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
    case 0x0501:
      updateSPI(peakValue, positionValue); // range is 2x 16bit
      break;
    case 0x0105:
      updateSPI(positionValue, peakValue);
      break;
    case 0x0505:
      updateSPI(peakValue, peakValue);
      break;
    case 0x0101:
      updateSPI(positionValue, positionValue);
      break;
    default:
      updateSPI(peakValue, positionValue); // range is 2x 16bit
      break;
    }

    memset((void *)adc0_buf, 0, sizeof(adc0_buf)); // clear DMA buffer

    // adc0_busy = 1;

    // update PGA
    adc->adc0->enablePGA(pga);
    adc->analogReadDifferential(A10, A11, ADC_0); // start ADC_0 differential - to use PGA
    // adc->analogRead(A10,ADC_0); // PGA is not working in single channel mode
    // adc0_dma.enable();
    adc->adc0->enableDMA();
    adc0_dma.enable();

    NVIC_DISABLE_IRQ(IRQ_PDB);
    // Serial.println("Start PDB");
    adc->adc0->startPDB(freq); // check ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);
    adc0_busy = true;
    updateResults(); // update outputs from adc_data[] during next ADC conversion
    holdingRegs[EXEC_TIME] = micros() - exectime;
  }
  else
  {
    holdingRegs[EXEC_TIME] = motorPulseIndex; // just for debugging
    // adc0_dma_isr();
  }
}

void adc0_dma_isr(void)
{
  adc0_dma.clearInterrupt();
  adc0_dma.clearComplete();
  // Serial.println("DMA interrupt");
  // PDB0_CH1C1 = 0; // clear PDB channel control register - should be implemented in stopPDB()
  PDB0_CH0C1 = 0;

  adc->adc0->stopPDB();
  adc0_dma.disable();
  adc->adc0->disableDMA();
  adc0_dma.disable();

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++) // copy DMA buffer
  {
    // adc_data[i]=(adc0_buf[i] + 256) >> 1; // full scale signal signed 9bit to unsigned 8bit
    if (adc0_buf[i] < 0) // only positive wave of signal 8bit
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
  // int peakValue = 0;
  int peak[ANALOG_BUFFER_SIZE] = {};
  long risingEdgeTime = 0;
  long fallingEdgeTime = 0;
  long peakValueTime = 0;
  // int peakValue = 0;
  int positionValueAvg = 0;

  peakValue = 0;
  positionValue = 0;

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
      peak[i] = adc_data[i]; // check first peak

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
        if ((positionMode == 4) && !peakValueTime)
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
    filterOnOff.interval(filterOn); // update filter interval
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
    case 4:
      positionValueDisp = peakValueTime;
      break;
    default:
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

  // warning! SPI makes noise, do not send data through SPI when ADC is running

  // switch (analogOutMode)
  // { // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
  // case 0x0501:
  //   updateSPI(peakValue, positionValue); // range is 2x 16bit
  //   break;
  // case 0x0105:
  //   updateSPI(positionValue, peakValue);
  //   break;
  // case 0x0505:
  //   updateSPI(peakValue, peakValue);
  //   break;
  // case 0x0101:
  //   updateSPI(positionValue, positionValue);
  //   break;
  // default:
  //   updateSPI(peakValue, positionValue); // range is 2x 16bit
  //   break;
  // }

  // if (dataSent && motorPulseIndex == 0) // prepare data for visualization on PC, only first mirror
  if (dataSent && motorPulseIndex == (filterPosition % 6)) // possibility to view different mirrors by changing positionFilter
  {
    for (int i = 0; i < 25; i++) // MOTOR_TIME_DIFF = AN_VALUES + 25
    {
      value_buffer[i] = (adc_data[(i * 8) + 4] % 256 << 8) | (adc_data[i * 8] % 256); // MSB = value_buffer[i*8+4] , LSB = value_buffer[i*8] ; only 50 of 200
      // value_buffer[i] = adc_data[i*8];
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
  holdingRegs[GAIN_SET1] = pga1 * 100;
  holdingRegs[THRESHOLD_SET1] = thre1 * 100;
  holdingRegs[GAIN_SET2] = pga2 * 100;
  holdingRegs[THRESHOLD_SET2] = thre2 * 100;

  holdingRegs[WINDOW_BEGIN] = windowBegin * 100;
  holdingRegs[WINDOW_END] = windowEnd * 100;
  holdingRegs[POSITION_MODE] = positionMode;
  holdingRegs[ANALOG_OUT_MODE] = analogOutMode;
  holdingRegs[POSITION_OFFSET] = positionOffset;

  holdingRegs[FILTER_POSITION] = filterPosition;
  holdingRegs[FILTER_ON] = filterOn;
  holdingRegs[FILTER_OFF] = filterOff;

  holdingRegs[ACT_TEMPERATURE] = celsius * 256;
  holdingRegs[MAX_TEMPERATURE] = max_temperature * 256;
  holdingRegs[TOTAL_RUNTIME] = total_runtime;
  holdingRegs[IO_STATE] = io_state;

  holdingRegs[MOTOR_TIME_DIFF] = motorTimeDiff;
  holdingRegs[OFFSET_DELAY] = delayOffset;

  // updated in updateResults()
  holdingRegs[PEAK_VALUE] = peakValueDisp * 100;               // 0-10000  0-100% * 100
  holdingRegs[POSITION_VALUE] = positionValueDisp * 10;        // 0-10000
  holdingRegs[POSITION_VALUE_AVG] = positionValueAvgDisp * 10; // 0-10000

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
    // modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, 1100, 0); // for compatibility with old SDIS
  }

  if (holdingRegs[SET] != set && holdingRegs[SET] < 4)
  { // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
    set = holdingRegs[SET];
    eeprom_writeInt(EE_ADDR_set, set);
  }

  if (holdingRegs[GAIN_SET1] != (pga1 * 100))
  {
    switch (holdingRegs[GAIN_SET1])
    { // check for valid values
    case 100:
    case 200:
    case 400:
    case 800:
    case 1600:
    case 3200:
    case 6400:
      pga1 = holdingRegs[GAIN_SET1] / 100;
      eeprom_writeInt(EE_ADDR_gain_set1, pga1);
      break;
    default:
      break;
    }
  }

  if ((holdingRegs[THRESHOLD_SET1] != (thre1 * 100)) && (holdingRegs[THRESHOLD_SET1] >= 2000) && (holdingRegs[THRESHOLD_SET1] <= 8000))
  {
    thre1 = holdingRegs[THRESHOLD_SET1] / 100;
    eeprom_writeInt(EE_ADDR_threshold_set1, thre1);
  }

  if (holdingRegs[GAIN_SET2] != (pga2 * 100))
  {
    switch (holdingRegs[GAIN_SET2])
    {
    case 100:
    case 200:
    case 400:
    case 800:
    case 1600:
    case 3200:
    case 6400:
      pga2 = holdingRegs[GAIN_SET2] / 100;
      eeprom_writeInt(EE_ADDR_gain_set2, pga2);
      break;
    default:
      break;
    }
  }

  if ((holdingRegs[THRESHOLD_SET2] != (thre2 * 100)) && (holdingRegs[THRESHOLD_SET2] >= 2000) && (holdingRegs[THRESHOLD_SET2] <= 8000))
  {
    thre2 = holdingRegs[THRESHOLD_SET2] / 100;
    eeprom_writeInt(EE_ADDR_threshold_set2, thre2);
  }

  if ((holdingRegs[WINDOW_BEGIN] != (windowBegin * 100)) && (holdingRegs[WINDOW_BEGIN] >= 500) && (holdingRegs[WINDOW_BEGIN] <= 4500))
  {
    windowBegin = holdingRegs[WINDOW_BEGIN] / 100;
    eeprom_writeInt(EE_ADDR_window_begin, windowBegin);
  }

  if ((holdingRegs[WINDOW_END] != (windowEnd * 100)) && (holdingRegs[WINDOW_END] >= 5500) && (holdingRegs[WINDOW_END] <= 9500))
  {
    windowEnd = holdingRegs[WINDOW_END] / 100;
    eeprom_writeInt(EE_ADDR_window_end, windowEnd);
  }

  if (holdingRegs[POSITION_MODE] != positionMode && holdingRegs[POSITION_MODE] < 5)
  {
    positionMode = holdingRegs[POSITION_MODE];
    eeprom_writeInt(EE_ADDR_position_mode, positionMode);
  }

  if (holdingRegs[ANALOG_OUT_MODE] != analogOutMode)
  {
    switch (holdingRegs[ANALOG_OUT_MODE]) // save if valid mode an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
    {
    case 0x0501:
    case 0x0105:
    case 0x0505:
    case 0x0101:
      analogOutMode = holdingRegs[ANALOG_OUT_MODE];
      eeprom_writeInt(EE_ADDR_analog_out_mode, analogOutMode);
      break;
    default:
      break;
    }
  }

  if (holdingRegs[POSITION_OFFSET] != positionOffset && holdingRegs[POSITION_OFFSET] >= 0 && holdingRegs[POSITION_OFFSET] <= 2000)
  {
    positionOffset = holdingRegs[POSITION_OFFSET];
    eeprom_writeInt(EE_ADDR_position_offset, positionOffset);

    // SCB_AIRCR = 0x05FA0004;        // software reset

    if (positionOffset < 1000) // depends on motor HALL sensors & mirror position - choose the best to have no timing issues
      attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, FALLING);
    else
      attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, RISING);
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
    { // check if IO_LASER bit is 00set
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

    if (holdingRegs[IO_STATE] & (1 << IO_SW_RESET))
    {
      SCB_AIRCR = 0x05FA0004;        // software reset
    }
  }
}

// check void ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);

// pdb interrupt is enabled in case you need it.
// void pdb_isr(void) {
//         PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt
//        // NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt
//         //digitalWriteFast(LED_BUILTIN,!digitalReadFast(LED_BUILTIN));
//         Serial.println("PDB interrupt");
// }

void InitVariables()
{
  modbusID = 1;

  actualSpeed = 3;     // array index
  modbusSpeed = 19200; // default 19200

  actualFormat = 1;
  modbusFormat = SERIAL_8N1;

  max_temperature = 0;

  dataSent = false;
  sendNextLn = 0;
  io_state = 0;
  exectime = 0;
  pulsetime = 0;

  total_runtime = 0;
}