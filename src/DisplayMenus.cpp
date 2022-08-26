#include <Arduino.h>
#include <LedDisplay.h>
#include <stdio.h>
#include <stdarg.h>

#include "Variables.h"
#include "EEPROMfunctions.h"
#include "DisplayMenus.h"
#include "SimpleModbusSlave.h"



volatile boolean alarmChecked = false;
volatile boolean extTest = false;
volatile boolean intTest = false;
volatile int currentMenu = MENU_MAIN;
volatile int currentMenuOption = 0;
volatile int btnHoldCounter = 0;



// Menu variables
volatile char lastKey = BTN_NONE; // Last key pressed
volatile int hourTimeout = 3600000;
volatile int refreshMenuTimeout = 0;
volatile int laserTimeout = 0;
volatile int testTimeout = 0;
volatile int menuTimeout = 0;

volatile int resultButtonA = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonB = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonC = STATE_NORMAL; // global value set by checkButton()
volatile int resultButtonD = STATE_NORMAL; // global value set by checkButton()

//menu login
int passwd = 0; // correct passwd is 1122
int nextBtn = 0;
boolean loggedIn = false;
volatile boolean blinkMenu = false;

const char *menu_modbusFormatDisp[] = {"8N1", "8E1", "8O1", "8N2"};
unsigned int menu_modbusID = 1;
unsigned int menu_modbusSpeed = 19200;
unsigned int menu_modbusFormat = SERIAL_8N1;

int menu_windowBegin, menu_windowEnd, menu_positionOffset, menu_positionMode, menu_analogOutMode;
int menu_filterPosition, menu_filterOn, menu_filterOff;
const char *menu_positionModeDisp[] = {"RISE", "FALL", "PEAK", " HMD"};
const char *menu_analogOutModeDisp[] = {"Pos", "Int"};


const char *menu_setDisp[] = {"MAN1", "MAN2", "REL ", "REL1", "REL2"};
int setDispIndex = 0;
int menu_pga;
int menu_thre;
int menu_set;

volatile long peakValueTimeDisp = 0;
volatile int peakValueDisp = 0;
volatile int positionValueDisp = 0;
volatile int positionValueAvgDisp = 0;

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
    case MENU_ANALOG_OUT1_MODE:
      setAnalogOut1Mode();
      break;
    case MENU_ANALOG_OUT2_MODE:
      setAnalogOut2Mode();
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
  case 1:
    setDispIndex = 0; // MAN1
    break;
  case 2:
    setDispIndex = 1; // MAN2
    break;
  case 3:
    setDispIndex = 2; // REL
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
    if (menu_set > 1)
      menu_set--;
    else
      menu_set = 3;
  }
  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_set < 3)
      menu_set++;
    else
      menu_set = 1;
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
    displayPrint("AnO1 %s", menu_analogOutModeDisp[analogOutMode >> 10]);
  if (currentMenuOption == 1)
    displayPrint("AnO2 %s", menu_analogOutModeDisp[(analogOutMode & 0xFF) >> 2]);
  if (currentMenuOption == 2)
    displayPrint("mPos%s", menu_positionModeDisp[positionMode-1]);
  if (currentMenuOption == 3)
    displayPrint("wBeg%3d%%", windowBegin);
  if (currentMenuOption == 4)
    displayPrint("wEnd%3d%%", windowEnd);
  if (currentMenuOption == 5)
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
      currentMenu = MENU_ANALOG_OUT1_MODE;
      currentMenuOption = 0;
      menu_analogOutMode = analogOutMode >> 10;
    }
    if (currentMenuOption == 1)
    {
      currentMenu = MENU_ANALOG_OUT2_MODE;
      currentMenuOption = 0;
      menu_analogOutMode = (analogOutMode & 0xFF) >> 2;
    }
    if (currentMenuOption == 2)
    {
      currentMenu = MENU_POSITION_MODE;
      currentMenuOption = 0;
      menu_positionMode = positionMode-1;
    }
    if (currentMenuOption == 3)
    {
      currentMenu = MENU_WINDOW_BEGIN;
      currentMenuOption = 0;
      menu_windowBegin = windowBegin;
    }
    if (currentMenuOption == 4)
    {
      currentMenu = MENU_WINDOW_END;
      currentMenuOption = 0;
      menu_windowEnd = windowEnd;
    }
    if (currentMenuOption == 5)
    {
      currentMenu = MENU_POSITION_OFFSET;
      currentMenuOption = 0;
      menu_positionOffset = positionOffset;
    }
  }
}


void setAnalogOut1Mode(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  
  if (blinkMenu)
    displayPrint("AnO1 %s", menu_analogOutModeDisp[menu_analogOutMode]);
  else
    displayPrint("     %s", menu_analogOutModeDisp[menu_analogOutMode]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_analogOutMode > 0)
      menu_analogOutMode--;
    else
      menu_analogOutMode = 1; // INT 5Hi
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_analogOutMode < 1)
      menu_analogOutMode++;
    else
      menu_analogOutMode = 0; // POS 1Hi
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 0;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    if (menu_analogOutMode) {analogOutMode |= (1 << 10);} else {analogOutMode &= ~(1 << 10);}  // set vs clear bit: 0x05XX vs 0x01XX
    eeprom_writeInt(EE_ADDR_analog_out_mode, analogOutMode); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 0;
  }
}

void setAnalogOut2Mode(void)
{
  if (!menuTimeout)
    menuTimeout = TIMEOUT_MENU;

  
  if (blinkMenu)
    displayPrint("AnO2 %s", menu_analogOutModeDisp[menu_analogOutMode]);
  else
    displayPrint("     %s", menu_analogOutModeDisp[menu_analogOutMode]);

  if (lastKey == BTN_B || lastKey == BTN_BH)
  {
    if (menu_analogOutMode > 0)
      menu_analogOutMode--;
    else
      menu_analogOutMode = 1; // INT 5Lo
  }

  if (lastKey == BTN_C || lastKey == BTN_CH)
  { // increment by 1
    if (menu_analogOutMode < 1)
      menu_analogOutMode++;
    else
      menu_analogOutMode = 0; // POS 1Lo
  }

  if (lastKey == BTN_A || lastKey == BTN_AH)
  {
    currentMenu = MENU_ANALOG;
    currentMenuOption = 1;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    if (menu_analogOutMode) {analogOutMode |= (1 << 2);} else {analogOutMode &= ~(1 << 2);}  // set vs clear bit: 0xXX05 vs 0xXX01
    eeprom_writeInt(EE_ADDR_analog_out_mode, analogOutMode); //save to EEPROM
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

  // positionMode: RISE = 1, FALL = 2, PEAK = 3, HMD = 4 
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
    positionMode = menu_positionMode+1;
    eeprom_writeInt(EE_ADDR_position_mode, positionMode); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 2;
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
    currentMenuOption = 3;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    windowBegin = menu_windowBegin;
    eeprom_writeInt(EE_ADDR_window_begin, windowBegin); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 3;
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
    currentMenuOption = 4;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    windowEnd = menu_windowEnd;
    eeprom_writeInt(EE_ADDR_window_end, windowEnd); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 4;
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
    currentMenuOption = 5;
  }

  if (lastKey == BTN_D || lastKey == BTN_DH)
  { // SAVE
    positionOffset = menu_positionOffset;
    //adc0_busy = 0;
    eeprom_writeInt(EE_ADDR_position_offset, positionOffset); //save to EEPROM
    displayPrint("SAVED!!!");
    delay(500);
    currentMenu = MENU_ANALOG;
    currentMenuOption = 5;
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
      checkSET();

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

