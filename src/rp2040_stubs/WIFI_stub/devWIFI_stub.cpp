// Temporary stand-in for lib/WIFI on RP2040 until WiFi mode is ported (Phase 5)
#include "targets.h"
#include "devWIFI.h"
#include "logging.h"

bool webserverPreventAutoStart = false;

void setWifiUpdateMode()
{
    // Don't enter wifiUpdate: it stops the radio and nothing would serve the web UI
    DBGLN("WiFi mode not supported on RP2040 yet");
}

device_t WIFI_device = {
    .initialize = nullptr,
    .start = nullptr,
    .event = nullptr,
    .timeout = nullptr,
    .subscribe = 0
};
