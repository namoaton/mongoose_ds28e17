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



