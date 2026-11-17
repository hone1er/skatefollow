#pragma once
#include <stdint.h>
// Shapes as in ESP-IDF 5.x (esp32 Arduino core 3.x)
typedef struct { uint8_t *src_addr; uint8_t *des_addr; void *rx_ctrl; } esp_now_recv_info_t;
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *info, const uint8_t *data, int data_len);
typedef struct { uint8_t peer_addr[6]; uint8_t lmk[16]; uint8_t channel; int ifidx; bool encrypt; void *priv; } esp_now_peer_info_t;
int esp_now_init();
int esp_now_register_recv_cb(esp_now_recv_cb_t);
int esp_now_add_peer(const esp_now_peer_info_t *);
int esp_now_send(const uint8_t *, const uint8_t *, size_t);
