#ifndef __DisplayMenus_H
#define __DisplayMenus_H

#include <Arduino.h>
#include <LedDisplay.h>

extern LedDisplay myDisplay;

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
#define MENU_ANALOG_OUT1_MODE 244
#define MENU_ANALOG_OUT2_MODE 245
#define MENU_POSITION_OFFSET 246

#define MENU_INFO 25
#define MENU_RESET 251

#define TIMEOUT_REFRESH_MENU 500
#define TIMEOUT_MENU 1200000 // *500us = 10 mins

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
void setAnalogOut1Mode(void);
void setAnalogOut2Mode(void);
void setPositionOffset(void);
void showInfoMenu(void);
void showResetMenu(void);

#endif /* __DisplayMenus_H */