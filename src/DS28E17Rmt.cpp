
#include "DS28E17Rmt.h"
#include <mgos_system.h>
#include <mgos_time.h>
#include "mgos.h"

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

bool DS28E17Rmt::check_status(uint8_t* status){
    uint8_t res = true;
    if((status[0]&0x01 )== 0x1)
    {
        res = false;
    }
    if((status[0]&0x02 )== 0x2)
    {
        res = false;
    }
    if((status[0]&0x04 )== 0x4)
    {
        res = false;
    }
    return  res;
}
uint16_t  DS28E17Rmt::calculateCrc16(uint16_t crc16, uint16_t data)
{
    const uint16_t oddparity[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
    data = (data ^ crc16) & 0xff;
    crc16 >>= 8;

    if (oddparity[data & 0xf] ^ oddparity[data >> 4])
    {
        crc16 ^= 0xc001;
    }

    data <<= 6;
    crc16 ^= data;
    data <<= 1;
    crc16 ^= data;

    return crc16;
}
uint16_t  DS28E17Rmt::crc16(uint8_t* input, uint16_t len, uint16_t  crc) {

    for (size_t i = 0; i < len; i++)
    {
        crc = calculateCrc16(crc, input[i]);
    }
    return crc;
}


uint16_t DS28E17Rmt::packet_crc(uint8_t* packet,uint16_t len)
{
    uint16_t  crc = 0;
    crc = crc16(packet, len, crc);
    crc =~crc;
    packet[len+1] = crc & 0xff;
    packet[len+2] = crc >>8;
    return len+3;
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

bool  DS28E17Rmt::WriteDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//    LOG(LL_WARN, ("WriteDataStop"));
    uint8_t  res = 1;
    uint8_t  status[2] = {0};
    uint8_t command[len + 5] = {Write_Data_Stop, i2c_addr, len};
    memcpy(&command[3],data,len );
   /* uint16_t  crc = 0;
    crc = crc16(command, len+3, crc);
    crc =~crc;
    command[len+4] = crc >>8;
    command[len+3] = crc & 0xff;*/
//    packet_crc(command,len+3);
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,packet_crc(command,len+3));// len+5
     mgos_msleep(5);
    _ow->read_bytes(status,2);
//    LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    b = _ow->reset();
    res = (b == 1);
    res =check_status(status);
    return  res;
}

bool  DS28E17Rmt::WriteDataNoStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//    LOG(LL_WARN, ("WriteDataNoStop"));
    uint8_t  res = 1;
    uint8_t  status[2] = {0};
    uint8_t command[len + 5] = {Write_Data_No_Stop, i2c_addr, len};
    memcpy(&command[3],data,len );
    uint16_t  crc = 0;
    crc = crc16(command, len+3, crc);
    crc =~crc;
    command[len+4] = crc >>8;
    command[len+3] = crc & 0xff;
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,len+5);
    mgos_msleep(5);
    _ow->read_bytes(status, 2);
    b = _ow->reset();
    res =  (b == 1);
//    LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    res =check_status(status);
    return  res;
}

bool  DS28E17Rmt::WriteDataOnlyStop(uint8_t* deviceAddress, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("WriteDataOnlyStop"));
    uint8_t  res = 1;
    uint8_t  status[2] = {0};
    uint8_t command[len + 4] = {Write_Data_Only_Stop, len};
    memcpy(&command[2],data,len );
    uint16_t  crc = 0;
    crc = crc16(command, len+2,crc);
    crc =~crc;
    command[len+3] = crc >>8;
    command[len+2] = crc & 0xff;
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,len+5);
     mgos_msleep(5);
    _ow->read_bytes(status, 2);
    b = _ow->reset();
    res =  (b == 1);
//  LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    res =check_status(status);
    return  res;
}
bool  DS28E17Rmt::WriteDataOnly(uint8_t* deviceAddress, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("WriteDataOnly"));
    uint8_t  res = 1;
    uint8_t  status[2] = {0};
    uint8_t command[len + 4] = {Write_Data_Only, len};
    memcpy(&command[2],data,len );
    uint16_t  crc = 0;
    crc = crc16(command, len+2,crc);
    crc =~crc;
    command[len+3] = crc >>8;
    command[len+2] = crc & 0xff;
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,len+5);
    mgos_msleep(5);
    _ow->read_bytes(status, 2);
    b = _ow->reset();
    res =  (b == 1);

//  LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    res =check_status(status);
    return  res;
}

bool  DS28E17Rmt::ReadDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("ReadDataStop"));
    bool res = true;
    uint8_t  status[2] = {0};
    uint8_t command[5] = {Read_Data_Stop, i2c_addr, len};
    uint16_t  crc = 0;
    crc = crc16(command, 3, crc);
    crc =~crc;
    command[4] = crc >>8;
    command[3] = crc & 0xff;
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,len+4);
    mgos_msleep(50);
    _ow->read_bytes(status, 1);
//    _ow->read_bytes(data, len);
//  LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    b = _ow->reset();
    res = (b == 1);
    res =check_status(status);
    return  res;
}
bool  DS28E17Rmt::WriteReadDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len_wr, uint8_t* data_wr,
                    uint8_t len_r, uint8_t * data_r){
//  LOG(LL_WARN, ("WriteReadDataStop"));
    bool res = true;
    uint8_t  status[2] = {0};
    uint8_t command[ len_wr + 6 ] = {Write_Read_Data_Stop, i2c_addr, len_wr};
    memcpy(&command[3],data_wr,len_wr );
    command[len_wr + 4] = len_r;
    uint16_t  crc = 0;
    crc = crc16(command, len_wr + 5, crc);
    crc =~crc;
    command[len_wr + 6] = crc >>8;
    command[len_wr + 5] = crc & 0xff;
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(command,len_wr + 5);
    mgos_msleep(50);
    _ow->read_bytes(status, 1);
    mgos_msleep(5);
    _ow->read_bytes(data_r, len_r);
//  LOG(LL_WARN, ("Status %X %X",status[0],status[1]));
    b = _ow->reset();
    res = (b == 1);
    res =check_status(status);
    return  res;
}
/*
 * 00b = I 2 C speed set to 100kHz
 * 01b = I 2 C speed set to 400kHz (power-on default)
 * 10b = I 2 C speed set to 900kHz
 * 11b = Not used
 */
bool   DS28E17Rmt::ReadConfig(uint8_t* deviceAddress, uint8_t * config){
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write(Read_Config);
    _ow->read_bytes(config, 1);
    b = _ow->reset();
    return (b == 1);
}

bool   DS28E17Rmt::WriteConfig(uint8_t* deviceAddress, uint8_t * config){
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write(Write_Config);
    _ow->write_bytes(config, 1);
    b = _ow->reset();
    return (b == 1);
}

bool   DS28E17Rmt::EnableSleep(uint8_t* deviceAddress){
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write(Enable_Sleep);
    b = _ow->reset();
    return (b == 1);
}
