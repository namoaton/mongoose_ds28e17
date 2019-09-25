
#include "DS28E17Rmt.h"
#include <mgos_system.h>
#include <mgos_time.h>
#include "mgos.h"
// OneWire commands
#define STARTCONVO \
  0x44  // Tells device to take a temperature reading and put it on the
        // scratchpad
#define COPYSCRATCH 0x48      // Copy EEPROM
#define READSCRATCH 0xBE      // Read EEPROM
#define WRITESCRATCH 0x4E     // Write to EEPROM
#define RECALLSCRATCH 0xB8    // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH 0xEC      // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB 0
#define TEMP_MSB 1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP 3
#define CONFIGURATION 4
#define INTERNAL_BYTE 5
#define COUNT_REMAIN 6
#define COUNT_PER_C 7
#define SCRATCHPAD_CRC 8

// Device resolution
#define TEMP_9_BIT 0x1F   //  9 bit
#define TEMP_10_BIT 0x3F  // 10 bit
#define TEMP_11_BIT 0x5F  // 11 bit
#define TEMP_12_BIT 0x7F  // 12 bit

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

DS28E17Rmt::DS28E17Rmt() : _ow(nullptr), _ownOnewire(false) {
}

DS28E17Rmt::DS28E17Rmt(OnewireRmt *ow) : _ownOnewire(false) {
  setOneWire(ow);
}

DS28E17Rmt::DS28E17Rmt(uint8_t pin, uint8_t rmt_rx, uint8_t rmt_tx)
    : _ow(new OnewireRmt(pin, rmt_rx, rmt_tx)), _ownOnewire(true) {
  _devices = 0;
  _parasite = false;
  _bitResolution = 9;
}

DS28E17Rmt::~DS28E17Rmt() {
  if (_ownOnewire) {
    delete _ow;
  }
}

void DS28E17Rmt::setOneWire(OnewireRmt *ow) {
  if (_ownOnewire) {
    delete _ow;
    _ow = nullptr;
  }
  _ow = ow;
  _devices = 0;
  _parasite = false;
}
// initialise the bus

void DS28E17Rmt::begin(void) {
  DeviceAddress deviceAddress;

  _ow->reset_search();
  _devices = 0;  // Reset the number of devices when we enumerate wire devices

  while (_ow->search(deviceAddress)) {
    if (validAddress(deviceAddress)) {
        LOG(LL_WARN, ("Found Bridge\n"));
      _devices++;
    }
  }
}
// returns true if address is valid

bool DS28E17Rmt::validAddress(const uint8_t *deviceAddress) {
    return(validFamily(deviceAddress));
}

bool DS28E17Rmt::validFamily(const uint8_t *deviceAddress) {
  bool ret;
  LOG(LL_WARN, ("Addr [0] = %X\n",deviceAddress[0]));
  switch (deviceAddress[0]) {
      case DS28E17MODEL:
      ret = true;
      break;
    default:
      ret = false;
  }
  return ret;
}

// finds an address at a given index on the bus
// returns true if the device was found

bool DS28E17Rmt::getAddress(uint8_t *deviceAddress, uint8_t index) {
  uint8_t depth = 0;

  _ow->reset_search();

  while (depth <= index && _ow->search(deviceAddress)) {
    if (depth == index && validAddress(deviceAddress)) return true;
    depth++;
  }

  return false;
}
bool  DS28E17Rmt::ReadDeviceRev(uint8_t* deviceAddress, uint8_t* rev){
    int b = _ow->reset();
    if (b == 0) return false;

    _ow->select(deviceAddress);
    _ow->write(Read_Device_Rev);
    _ow->read_bytes(rev, 1);

    b = _ow->reset();
    return (b == 1);
}


