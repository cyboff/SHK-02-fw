// EEPROM functions and definitions not included in standart <EEPROM.h> library
#ifndef __EEPROMfunctions_H
#define __EEPROMfunctions_H

//defaults EEPROM
#define MODEL_TYPE 50
#define MODEL_SERIAL_NUMBER 23001
#define FW_VERSION 232           

#define DEFAULT_MODBUS_ID MODEL_SERIAL_NUMBER % 1000 % 247 // MODBUS ID slave (range 1..247)
#define DEFAULT_MODBUS_SPEED 19200
#define DEFAULT_MODBUS_FORMAT SERIAL_8N1

#define DEFAULT_SET 3             // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
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

#define DEFAULT_POSITION_MODE 4     // rising = 1, falling = 2, peak = 3, hmd = 4
#define DEFAULT_ANALOG_OUT_MODE 0x0501   // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
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
#define EE_ADDR_position_mode 0x26   // WORD  // positionMode: RISE = 1, FALL = 2, PEAK = 3, HMD = 4
#define EE_ADDR_analog_out_mode 0x28 // WORD  // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
#define EE_ADDR_position_offset 0x30 // WORD  // offset for position

#define EE_ADDR_filter_position 0x32 // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_on 0x34       // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_off 0x36      // WORD  // range 0 - 9999 ms

// EEPROM Addresses for diagnosis
#define EE_ADDR_max_temperature 0x38 // WORD
#define EE_ADDR_total_runtime 0x40   // WORD

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(unsigned int address, unsigned int value);
void eeprom_updateInt(unsigned int address, unsigned int value);
// read a unsigned int (two bytes) value from eeprom
unsigned int eeprom_readInt(unsigned int address);
void EEPROM_init();
void config_loadFromEEPROM();
void config_writeDefaultsToEEPROM();
void reset_writeDefaultsToEEPROM();

#endif