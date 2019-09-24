# Library to read Dallas temperature sensor using ESP32's RMT device
The library uses the `onewire-rmt` library. 
A pullup resistor (4.7k) must be connected between the data line of the sensor and Vcc.

# Example code in C++
```
#include <mgos.h>
#include "DallasRmt.h"

static DallasRmt* dallas = NULL;
static OnewireRmt* ow = NULL;
static int numDevices = 0;
static uint32_t readings = 0;

static void loop_onewire(void *arg)
{
    (void) arg;
    if (0 == numDevices) {
        LOG(LL_WARN, ("No device found"));
        return;
    }
    bool wait = dallas->getWaitForConversion();
    double start = mg_time();
    dallas->requestTemperatures();
    double end = mg_time();
    LOG(LL_WARN, ("wait=%d, conversionTime=%.2fms", wait, 1000.0 * (end - start)));

    uint8_t deviceAddress[8];
    for (int i = 0; i < numDevices; ++i) {
        float temp = dallas->getTempCByIndex(i);
        dallas->getAddress(deviceAddress, i);
        char buf[17];
        char*p = buf;
        for (int i = 0; i < 8; ++i) {
            p += snprintf(p, sizeof (buf) - 2 * i, "%02x", deviceAddress[i]);
        }
        LOG(LL_WARN, ("index=%d, address=%s, temp=%.2f, reading=%d", i, buf, temp, readings));
        readings++;
    }
}

extern "C"
enum mgos_app_init_result mgos_app_init(void)
{
    dallas = new DallasRmt();
    ow = new OnewireRmt(13 /*pin*/, 0 /*rmt_rx*/, 1 /*rmt_tx*/);
    dallas->setOneWire(ow);
    dallas->begin();
    numDevices = dallas->getDeviceCount();
    
    mgos_set_timer(3000, 1, loop_onewire, NULL);

    return MGOS_APP_INIT_SUCCESS;
}
```

# Example code in C
```
#include <mgos.h>
#include "mgos_dallas_rmt.h"

static DallasRmt* dallas = NULL;
static int numDevices = 0;
static uint32_t readings = 0;

static void readData() {
    char deviceAddress[8];
    for (int i = 0; i < numDevices; ++i) {
        float temp = mgos_dallas_rmt_get_tempc_by_index(dallas, i) / 100.0;
        mgos_dallas_rmt_get_address(dallas, deviceAddress, i);
        char buf[17];
        char*p = buf;
        for (int i = 0; i < 8; ++i) {
            p += snprintf(p, sizeof (buf) - 2 * i, "%02x", deviceAddress[i]);
        }
        LOG(LL_WARN, ("index=%d, address=%s, temp=%.2f, reading=%d", i, buf, temp, readings));
        readings++;
    }
}

static void owTimerCB(void * arg) {
    readData();
}

static void loop_onewire(void *arg) {
    (void) arg;
    if (0 == numDevices) {
        LOG(LL_WARN, ("No device found"));
        return;
    }
    bool wait = mgos_dallas_rmt_get_wait_for_conversion(dallas);
    double start = mg_time();
    mgos_dallas_rmt_request_temperatures(dallas);
    double end = mg_time();
    LOG(LL_WARN, ("wait=%d, conversionTime=%.2fms", wait, 1000.0 * (end - start)));
    if (wait) {
        readData();
    } else {
        mgos_set_timer(800, 0, owTimerCB, NULL);
    }
}

enum mgos_app_init_result mgos_app_init(void) {
    OnewireRmt* ow = mgos_onewire_rmt_create(13 /*pin*/, 0 /*rmt_rx*/, 1 /*rmt_tx*/);
    dallas = mgos_dallas_rmt_create(ow);
    
    mgos_dallas_rmt_begin(dallas);
    numDevices = mgos_dallas_rmt_get_device_count(dallas);
    
    int resolution = mgos_dallas_rmt_get_global_resolution(dallas);
    int16_t millis = mgos_dallas_rmt_millis_to_wait_for_conversion(dallas, resolution);
    LOG(LL_WARN, ("resolution=%d, millis=%d", resolution, millis));
    //mgos_dallas_rmt_set_wait_for_conversion(dallas, false);

    mgos_set_timer(3000, 1, loop_onewire, NULL);

    return MGOS_APP_INIT_SUCCESS;
}


