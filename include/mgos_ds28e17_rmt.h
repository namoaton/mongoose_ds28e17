#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "mgos_onewire_rmt.h"
#ifdef __cplusplus
#include "DS28E17Rmt.h"
#else
typedef struct DS28E17RmtTag DS28E17Rmt;
typedef struct OnewireRmtTag OnewireRmt;
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* Initializes the DS28E17Rmt driver with a `OnewireRmt*` object.
 * Return value: handle opaque pointer.
 */
DS28E17Rmt *mgos_ds28e17_rmt_create(OnewireRmt *ow);

/* Initializes the DS28E17Rmt driver with a GPIO `pin`, the RMT receive channel
 * `rmt_rx`
 * and the RMT transmit channel `rmt_tx`.
 * Return value: handle opaque pointer.
 */
DS28E17Rmt *mgos_ds28e17_rmt_create_with_channels(uint8_t pin, uint8_t rmt_rx,
                                                uint8_t rmt_tx);

/*
 *  Close DS28E17Rmt handle. Return value: none.
 */
void mgos_ds28e17_rmt_close(DS28E17Rmt *dt);

/*
 *  Initialises the 1-Wire bus.
 */
void mgos_ds28e17_rmt_begin(DS28E17Rmt *dt);

/*
 * Returns the number of devices found on the bus.
 * Return always 0 if an operaiton failed.
 */
int mgos_ds28e17_rmt_get_device_count(DS28E17Rmt *dt);

/*
 * Returns true if address is valid.
 * Return always false if an operaiton failed.
 */
bool mgos_ds28e17_rmt_valid_address(DS28E17Rmt *dt, const char *addr);

/*
 * Returns true if address is of the family of sensors the lib supports.
 * Return always false if an operaiton failed.
 */
bool mgos_ds28e17_rmt_valid_family(DS28E17Rmt *dt, const char *addr);

/*
 * Finds an address at a given index on the bus.
 * Return false if the device was not found or an operaiton failed.
 * Returns true otherwise.
 */
bool mgos_ds28e17_rmt_get_address(DS28E17Rmt *dt, char *addr, int idx);

/*
 * Read bridge revision
 */
bool mgos_ds28e17_rmt_read_device_rev(DS28E17Rmt *dt, char *addr, uint8_t *rev);
/*
 * Write data stop to I2c
 */
bool mgos_ds28e17_rmt_write_data_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data);
bool mgos_ds28e17_rmt_write_data_no_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data);
bool mgos_ds28e17_rmt_write_data_only_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t len, uint8_t* data);
bool mgos_ds28e17_rmt_write_data_only(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t len, uint8_t* data);
bool mgos_ds28e17_rmt_read_data_stop(DS28E17Rmt *dt,uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len, uint8_t* data);
bool mgos_ds28e17_rmt_write_read_data_stop(DS28E17Rmt *dt, uint8_t* deviceAddress, uint8_t i2c_addr, uint8_t len_wr, uint8_t* data_wr, uint8_t len_r, uint8_t * data_r);
/*
 * Read bridge config
 */

bool mgos_ds28e17_rmt_read_device_config(DS28E17Rmt *dt, char *addr, uint8_t *config);
/*
 * Write bridge config
 */

bool mgos_ds28e17_rmt_write_device_config(DS28E17Rmt *dt, char *addr, uint8_t *config);
/*
 * Enter sleep mode
 */

bool mgos_ds28e17_rmt_enable_sleep(DS28E17Rmt *dt, char *addr);

#ifdef __cplusplus
}
#endif
