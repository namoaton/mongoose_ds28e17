#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "OnewireRmt.h"

/***************DS28E17 commands***************************/
#define Write_Data_Stop         0x4B
#define Write_Data_No_Stop      0x5A
#define Write_Data_Only         0x69
#define Write_Data_Only_Stop    0x78
#define Read_Data_Stop          0x87
#define Write_Read_Data_Stop    0x2D
#define Write_Config            0xD2
#define Read_Config             0xE1
#define Enable_Sleep            0x1E
#define Read_Device_Rev         0xC3
// Model IDs
#define DS28E17MODEL 0x19  //
// Error Codes
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040

typedef uint8_t DeviceAddress[8];

class DS28E17Rmt {
 public:
  DS28E17Rmt();
  DS28E17Rmt(OnewireRmt *ow);
  DS28E17Rmt(uint8_t pin, uint8_t rmt_rx, uint8_t rmt_tx);

  ~DS28E17Rmt();

  void setOneWire(OnewireRmt *ow);

  /*
   * Initialises the bus
   */
  void begin(void);

  /*
   *  Returns the number of devices found on the bus
   */
  uint8_t getDeviceCount(void) {
    return _devices;
  }

  /*
   *  Returns true if address is valid
   */
  bool validAddress(const uint8_t *);

  /*
   * Returns true if address is of the family of sensors the lib supports.
   */
  bool validFamily(const uint8_t *deviceAddress);

  /*
   * Finds an address at a given index on the bus
   */
  bool getAddress(uint8_t *deviceAddress, uint8_t index);
  uint16_t crc16(uint8_t* input, uint16_t len);
  bool  ReadDeviceRev(uint8_t* deviceAddress, uint8_t* rev);
  bool WriteDataStop(uint8_t* deviceAddress, uint8_t  i2c_addr, uint8_t len, uint8_t* data);

 private:
  /*
   * The OneWire object
   */
  OnewireRmt *_ow;

  /*
   * count of devices on the bus
   */
  uint8_t _devices;

  /*
   * Parasite power on or off
   */
  bool _parasite;

  /*
   * Used to determine the delay amount needed to allow for the
   * temperature conversion to take place
   */
  uint8_t _bitResolution;

  bool _waitForConversion;
  bool _checkForConversion;

  /*
   * Set to true if we created _ow
   */
  bool _ownOnewire;

};
