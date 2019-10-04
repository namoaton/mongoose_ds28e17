#include <mgos.h>
#include "mgos_ds28e17_rmt.h"
#include "OnewireRmt.h"

DS28E17Rmt *mgos_ds28e17_rmt_create(OnewireRmt *ow) {
  if (ow == nullptr) return nullptr;
  return new DS28E17Rmt(ow);
}

DS28E17Rmt *mgos_ds28e17_rmt_create_with_channels(uint8_t pin, uint8_t rmt_rx,
                                                uint8_t rmt_tx) {
  return new DS28E17Rmt(pin, rmt_rx, rmt_tx);
}

void mgos_ds28e17_rmt_close(DS28E17Rmt *dt) {
  if (dt != nullptr) {
    delete dt;
    dt = nullptr;
  }
}

void mgos_ds28e17_rmt_set_global_ds(DS28E17Rmt *ds){
    globalDS = ds;
}

void mgos_ds28e17_rmt_set_addr(uint8_t* addr){
    DS28E17Rmt * dt =mgos_ds28e17_rmt_get_global_ds();
    dt->setDeviceAddress(addr);
}

uint8_t *mgos_ds28e17_rmt_get_addr(){
    DS28E17Rmt * dt =mgos_ds28e17_rmt_get_global_ds();
    return dt->deviceAddress;
}
DS28E17Rmt* mgos_ds28e17_rmt_get_global_ds(){
    return globalDS;
}
void mgos_ds28e17_rmt_begin(DS28E17Rmt *dt) {
  if (dt == nullptr) return;
  dt->begin();
}

int mgos_ds28e17_rmt_get_device_count(DS28E17Rmt *dt) {
  if (dt == nullptr) return 0;
  return dt->getDeviceCount();
}

bool mgos_ds28e17_rmt_valid_address(DS28E17Rmt *dt, const char *addr) {
  if (dt == nullptr) return false;
  return dt->validAddress((uint8_t *) addr);
}

bool mgos_ds28e17_rmt_valid_family(DS28E17Rmt *dt, const char *addr) {
  if (dt == nullptr) return false;
  return dt->validFamily((uint8_t *) addr);
}

bool mgos_ds28e17_rmt_get_address(DS28E17Rmt *dt, char *addr, int idx) {
  if (dt == nullptr) return false;
  return dt->getAddress((uint8_t *) addr, idx);
}
bool mgos_ds28e17_rmt_read_device_rev(DS28E17Rmt *dt, char *addr,uint8_t *rev) {
    if (dt == nullptr) return false;
    return dt->ReadDeviceRev((uint8_t *) addr, rev);
}
bool mgos_ds28e17_rmt_write_data_stop(DS28E17Rmt *dt,uint8_t* deviceAddress,uint8_t i2c_addr, uint8_t len, uint8_t* data){
    if (dt == nullptr) return false;
    return dt->WriteDataStop((uint8_t *) deviceAddress, i2c_addr,len, data);
}

bool mgos_ds28e17_rmt_write_data_no_stop(DS28E17Rmt *dt,uint8_t* deviceAddress,uint8_t i2c_addr, uint8_t len, uint8_t* data){
    if (dt == nullptr) return false;
    return dt->WriteDataNoStop((uint8_t *) deviceAddress, i2c_addr,len, data);
}

bool mgos_ds28e17_rmt_write_data_only_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t len, uint8_t* data){
    if (dt == nullptr) return false;
    return dt->WriteDataOnlyStop((uint8_t *) deviceAddress,len, data);
}

bool mgos_ds28e17_rmt_write_data_only(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t len, uint8_t* data){
    if (dt == nullptr) return false;
    return dt->WriteDataOnly((uint8_t *) deviceAddress,len, data);
}

bool mgos_ds28e17_rmt_read_data_stop(DS28E17Rmt *dt,uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data){
    if (dt == nullptr) return false;
    return dt->ReadDataStop((uint8_t *) deviceAddress, i2c_addr,len, data);
}
bool mgos_ds28e17_rmt_write_read_data_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len_wr, uint8_t* data_wr, uint8_t len_r, uint8_t * data_r){
    if (dt == nullptr) return false;
    return dt->WriteReadDataStop((uint8_t *) deviceAddress, i2c_addr,len_wr, data_wr,len_r,data_r);
}
bool mgos_ds28e17_rmt_read_device_config(DS28E17Rmt *dt, char *addr, uint8_t *config){
    if (dt == nullptr) return false;
    return dt->ReadConfig((uint8_t *) addr, config);
}

bool mgos_ds28e17_rmt_write_device_config(DS28E17Rmt *dt, char *addr, uint8_t *config){
    if (dt == nullptr) return false;
    return dt->WriteConfig((uint8_t *) addr, config);
}

bool mgos_ds28e17_rmt_enable_sleep(DS28E17Rmt *dt, char *addr){
    if (dt == nullptr) return false;
    return dt->EnableSleep((uint8_t *) addr);
}

