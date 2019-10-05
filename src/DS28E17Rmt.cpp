
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

void DS28E17Rmt::setDeviceAddress(uint8_t* dev_addr){
    memcpy(deviceAddress,dev_addr,8);
}
uint8_t* DS28E17Rmt::getDeviceAddress(){
    return deviceAddress;
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
bool DS28E17Rmt::ow_write_command(uint8_t* deviceAddress, uint8_t command){
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write(command);
    b = _ow->reset();
    return (b == 1);
}
bool DS28E17Rmt::ow_read_byte(uint8_t* deviceAddress,  uint8_t command, uint8_t *byte){
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write(command);
    _ow->read_bytes(byte, 1);
    b = _ow->reset();
    return (b == 1);
}
bool  DS28E17Rmt::ow_write_byte(uint8_t* deviceAddress, uint8_t command, uint8_t *byte){
        int b = _ow->reset();
        if (b == 0) return false;
        _ow->select(deviceAddress);
        _ow->write(command);
        _ow->write_bytes(byte, 1);
        b = _ow->reset();
        return (b == 1);
}
bool DS28E17Rmt::ow_write_bytes(uint8_t* deviceAddress, uint8_t len, uint8_t *bytes){
    uint8_t  res = 1;
    uint8_t  status[2] = {0};
    uint8_t  cmd_crc[len+2];
    memcpy(cmd_crc,bytes,len);
    packet_crc(bytes,len);
    int b = _ow->reset();
    if (b == 0) return false;
    _ow->select(deviceAddress);
    _ow->write_bytes(cmd_crc,len+2);
    mgos_msleep(5);
    _ow->read_bytes(status, 2);
    b = _ow->reset();
    res =  (b == 1);
    res =check_status(status);
    return  res;
}
bool  DS28E17Rmt::ow_read_bytes(uint8_t* deviceAddress, uint8_t *command, uint8_t len_w, uint8_t *bytes, uint8_t len_r){
    bool res = true;
    uint8_t  status[2] = {0};
    uint8_t  read_buffer[len_r]={0};
    uint8_t  cmd_crc[len_w+2];
    memcpy(cmd_crc,command,len_w);
    packet_crc(cmd_crc,len_w);
//    packet_crc(command,len_w);
    int b = _ow->reset();
    if (b == 0) return false;
//    for (int i =0;i<len_w+4;i++){
//        LOG(LL_WARN, ("cmd[%d] = %X",i,cmd_crc[i]));
//    }
    _ow->select(deviceAddress);
    _ow->write_bytes(cmd_crc,len_w + 2);
    mgos_msleep(10);
    _ow->read_bytes(status, 1);
    mgos_msleep(1);
    _ow->read_bytes(read_buffer, len_r);
    b = _ow->reset();
//    memcpy(status,read_bytes,1);
//    memcpy(bytes,&read_bytes[1],len_r);
    res = (b == 1);
    res =check_status(status);

    uint8_t last_bit = 0;
    for(int f=len_r-1;f>=0;f--){
        bytes[f] =(read_buffer[f]>>1)|last_bit<<7;
        last_bit =read_buffer[f] & 1;
    }
//     if (len_r>1) {
//         for (int i = 0; i < len_r; i++) {
//             LOG(LL_WARN, ("buffer[%d] = %X", i, bytes[i]));
//         }
//     }
    return  res;
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
    if((status[0]&0x01 )== 0x1) res = false;
    if((status[0]&0x02 )== 0x2) res = false;
    if((status[0]&0x04 )== 0x4) res = false;
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
    packet[ len ] = crc & 0xff;
    packet[ len + 1 ] = crc >>8;
    return len + 2;
}

bool  DS28E17Rmt::ReadDeviceRev(uint8_t* deviceAddress, uint8_t* rev){
    return ow_read_byte(deviceAddress, Read_Device_Rev, rev);
}

bool  DS28E17Rmt::WriteDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//    LOG(LL_WARN, ("WriteDataStop"));
    uint8_t command[len + 5] = {Write_Data_Stop, i2c_addr, len};
    memcpy(&command[3],data,len );
    return ow_write_bytes(deviceAddress,len+3, command);
}

bool  DS28E17Rmt::WriteDataNoStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//    LOG(LL_WARN, ("WriteDataNoStop"));
    uint8_t command[len + 5] = {Write_Data_No_Stop, i2c_addr, len};
    memcpy(&command[3],data,len );
    return ow_write_bytes(deviceAddress,len+3, command);

}

bool  DS28E17Rmt::WriteDataOnlyStop(uint8_t* deviceAddress, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("WriteDataOnlyStop"));
    uint8_t command[len + 4] = {Write_Data_Only_Stop, len};
    memcpy(&command[2],data,len );
    return ow_write_bytes(deviceAddress,len+2, command);
}
bool  DS28E17Rmt::WriteDataOnly(uint8_t* deviceAddress, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("WriteDataOnly"));
    uint8_t command[len + 4] = {Write_Data_Only, len};
    memcpy(&command[2],data,len );
    return ow_write_bytes(deviceAddress,len+2, command);
}

bool  DS28E17Rmt::ReadDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
//  LOG(LL_WARN, ("ReadDataStop"));
    uint8_t command[5] = {Read_Data_Stop, i2c_addr, len};
    return ow_read_bytes(deviceAddress, command,  3, data, len);
}
bool  DS28E17Rmt::WriteReadDataStop(uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len_wr, uint8_t* data_wr,
                    uint8_t len_r, uint8_t * data_r){
//  LOG(LL_WARN, ("WriteReadDataStop"));
    uint8_t command[ len_wr + 4 ] = {Write_Read_Data_Stop, i2c_addr, len_wr};
    memcpy(&command[3],data_wr,len_wr );
    command[len_wr + 3] = len_r;
    return ow_read_bytes(deviceAddress, command,  len_wr+4, data_r, len_r);
}
/*
 * 00b = I 2 C speed set to 100kHz
 * 01b = I 2 C speed set to 400kHz (power-on default)
 * 10b = I 2 C speed set to 900kHz
 * 11b = Not used
 */
bool   DS28E17Rmt::ReadConfig(uint8_t* deviceAddress, uint8_t * config){
    return ow_read_byte(deviceAddress, Read_Config, config);
}

bool   DS28E17Rmt::WriteConfig(uint8_t* deviceAddress, uint8_t * config){
    return  ow_write_byte(deviceAddress,Write_Config, config);

}

bool   DS28E17Rmt::EnableSleep(uint8_t* deviceAddress){
    return  ow_write_command(deviceAddress,Enable_Sleep);
}
