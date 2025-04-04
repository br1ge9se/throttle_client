#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include "board_conf.h"
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define MY_SERVICE_UUID 0xd4, 0x86, 0x48, 0x24, 0x54, 0xB3, 0x43, 0xA1, \
			            0xBC, 0x20, 0x97, 0x8F, 0xC3, 0x76, 0xC2, 0x75

#define RX_CHARACTERISTIC_UUID  0xA6, 0xE8, 0xC4, 0x60, 0x7E, 0xAA, 0x41, 0x6B, \
			                    0x95, 0xD4, 0x9D, 0xCC, 0x08, 0x4F, 0xCF, 0x6A

#define TX_CHARACTERISTIC_UUID  0xED, 0xAA, 0x20, 0x11, 0x92, 0xE7, 0x43, 0x5A, \
			                    0xAA, 0xE9, 0x94, 0x43, 0x35, 0x6A, 0xD4, 0xD3


#define u8_t uint8_t
#define u16_t uint16_t
#define u32_t uint32_t

#ifdef DONGLE
	extern void zprintf(const char *fmt, ...);
#else
	#define zprintf printk
#endif

/** @brief Callback type for when new data is received. */
typedef void (*data_rx_cb_t)(uint8_t *data, uint8_t length);

/** @brief Callback struct used by the my_service Service. */
struct my_service_cb 
{
	/** Data received callback. */
	data_rx_cb_t    data_rx_cb;
};

int my_service_init(void);

void my_service_send(struct bt_conn *conn, const uint8_t *data, uint16_t len);