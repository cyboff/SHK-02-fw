#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

/*
 SimpleModbusSlave allows you to communicate
 to any slave using the Modbus RTU protocol.
 
 The crc calculation is based on the work published 
 by jpmzometa at 
 http://sites.google.com/site/jpmzometa/arduino-mbrt
 
 By Juan Bester : bester.juan@gmail.com
 
 The functions implemented are functions 3 and 16.
 read holding registers and preset multiple registers
 of the Modbus RTU Protocol, to be used over the Arduino serial connection.
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.
 
 These library of functions are designed to enable a program send and
 receive data from a device that communicates using the Modbus protocol.
 
 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.
 
 This code is for a Modbus slave implementing functions 3 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 16: Presets values into a sequence of holding registers (4X references)
 
 All the functions share the same register array.
 
 Exception responses:
 1 ILLEGAL FUNCTION
 2 ILLEGAL DATA ADDRESS
 3 ILLEGAL DATA VALUE
 
 Note:  
 The Arduino serial ring buffer is 128 bytes or 64 registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 118 bytes or 59 registers.
 
 Using the FTDI converter ic the maximum bytes you can send is limited 
 to its internal buffer which is 60 bytes or 30 unsigned int registers. 
 
 Thus:
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 54 bytes or 27 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 50 bytes or 25 registers.
  
 Since it is assumed that you will mostly use the Arduino to connect to a 
 master without using a USB to Serial converter the internal buffer is set
 the same as the Arduino Serial ring buffer which is 128 bytes.
 
 The functions included here have been derived from the 
 Modbus Specifications and Implementation Guides
 
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf
*/

#include "Arduino.h"

// function definitions
// void modbus_configure(long baud, unsigned char _slaveID, unsigned char _TxEnablePin, uint16_t _holdingRegsSize, unsigned char _lowLatency)
void modbus_configure(long baud, uint16_t format, byte _slaveID, byte _TxEnablePin, uint16_t _holdingRegsSize, unsigned char _lowLatency);
uint16_t modbus_update(uint16_t *holdingRegs);

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

#endif
