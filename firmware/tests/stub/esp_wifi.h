#pragma once
#include <stdint.h>
enum wifi_second_chan_t { WIFI_SECOND_CHAN_NONE };
int esp_wifi_set_channel(uint8_t, wifi_second_chan_t);
