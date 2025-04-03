/*
 * Copyright (c) 2023 Br1Ge9se
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief BT Throttle peripheral app
 *
 * App for transmitting the ADC potentiometer value to the BLE Central.
 * USB CDC ACM is used as a terminal console.
 *
 */

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <dk_buttons_and_leds.h>
#include <soc.h>
#include <nrfx_saadc.h>
#include "saadc_examples_common.h"
#include "nrfx_example.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/reboot.h>

#include "my_service.h"

/*****************************************************************************************
 *
 *
 *****************************************************************************************/
#define NUM_OF_LEDS 4
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)
/* The devicetree node identifier for the "led2" alias. */
#define LED2_NODE DT_ALIAS(led2)
/* The devicetree node identifier for the "led3" alias. */
#define LED3_NODE DT_ALIAS(led3)

static const struct gpio_dt_spec led[NUM_OF_LEDS] = 
{
	GPIO_DT_SPEC_GET(LED0_NODE, gpios),
	GPIO_DT_SPEC_GET(LED1_NODE, gpios),
	GPIO_DT_SPEC_GET(LED2_NODE, gpios),
	GPIO_DT_SPEC_GET(LED3_NODE, gpios)
};


#define ANALOG_INPUT_A5 5

/** @brief Symbol specifying analog input to be observed by SAADC channel 5. */
#define CH5_AIN ANALOG_INPUT_TO_SAADC_AIN(ANALOG_INPUT_A5)

/**
 * Button to restart sys
 */
#define KEY_READVAL_MASK DK_BTN1_MSK

/** @brief Declaration of enum containing a set of states for the simple state machine. */
typedef enum
{
    STATE_SINGLE_CONFIG,     ///< Configure a single SAADC channel and set the SAADC driver in the simple mode.
    STATE_SINGLE_SAMPLING,   ///< Trigger SAADC sampling on the single channel.
} adc_state_t;

/** @brief SAADC channel configuration structure for channel use. */
static const nrfx_saadc_channel_t m_single_channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(CH5_AIN, 0);

/** @brief Buffer to store channel's values. */
static nrf_saadc_value_t m_samples_buffer;

/** @brief Symbol specifying the number of SAADC samplings to trigger. */
#define SAMPLING_ITERATIONS 256

/** @brief Enum with the current state of the simple state machine. */
static adc_state_t m_current_state = STATE_SINGLE_CONFIG;

/** @brief Flag indicating that sampling on every specified channel is finished and buffer ( @ref m_samples_buffer ) is filled with samples. */
static bool m_saadc_ready;

static uint32_t saadc_curr_value = 0;

#ifdef DONGLE
	static const struct device *console_dev = NULL;

	extern void zprintf(const char *fmt, ...);
	extern void zprint_init(const struct device *dev);
#else
	#define zprintf printk
#endif

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MY_SERVICE_UUID),
};

struct bt_conn *my_connection;

// LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

uint8_t led2blk;

USBD_DEVICE_DEFINE(sample_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0001);
/*****************************************************************************************
 *
 *
 *****************************************************************************************/

/**
 * @brief Key pressed handler
 *
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	uint32_t button = button_state & has_changed;

	if (button & KEY_READVAL_MASK) {
		
		zprintf("!!! System restart !!!");
		k_sleep(K_MSEC(500));
		sys_reboot(SYS_REBOOT_WARM);
	}
}
/**
 * @brief Function for handling SAADC driver events.
 *
 * @param[in] p_event Pointer to an SAADC driver event.
 */
static void saadc_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t status;
    (void)status;
	
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_DONE:
 //           zprintf("[Sample %d] value == %d\n\r", cnt_conv++, p_event->data.done.p_buffer[0]);
            saadc_curr_value = p_event->data.done.p_buffer[0];
            m_saadc_ready = true;
            break;

        case NRFX_SAADC_EVT_CALIBRATEDONE:
            status = nrfx_saadc_mode_trigger();
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        default:
            break;
    }
}

/**
 * @brief Function for 
 *
 */
static void saadc_notify(void)
{
	uint32_t sampling_index = 0;
	nrfx_err_t status;

	switch (m_current_state)
	{
		case STATE_SINGLE_CONFIG:
			status = nrfx_saadc_channel_config(&m_single_channel);
			NRFX_ASSERT(status == NRFX_SUCCESS);

			uint32_t channels_mask = nrfx_saadc_channels_configured_get();
			status = nrfx_saadc_simple_mode_set(channels_mask,
												NRF_SAADC_RESOLUTION_8BIT,
												NRF_SAADC_OVERSAMPLE_8X,
												saadc_handler);
			NRFX_ASSERT(status == NRFX_SUCCESS);

			status = nrfx_saadc_buffer_set(&m_samples_buffer, 1);
			NRFX_ASSERT(status == NRFX_SUCCESS);

			m_saadc_ready = true;
			m_current_state = STATE_SINGLE_SAMPLING;
			break;

		case STATE_SINGLE_SAMPLING:
			if (m_saadc_ready && sampling_index < SAMPLING_ITERATIONS)
			{
				status = nrfx_saadc_mode_trigger();
				NRFX_ASSERT(status == NRFX_SUCCESS);
				m_saadc_ready = false;
				sampling_index++;
			}
			else if (m_saadc_ready && sampling_index == SAMPLING_ITERATIONS)
			{
				status = nrfx_saadc_offset_calibrate(saadc_handler);
				NRFX_ASSERT(status == NRFX_SUCCESS);
				m_saadc_ready = false;
				sampling_index = 0;
			}
			break;

		default:
			break;
	}
}

#ifdef DONGLE
/**
 * @brief Uart Interrupt Handler
 *
 * @param[in] 
 */
static void uart_interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
//				LOG_ERR("Failed to read UART FIFO");
				zprintf("Failed to read UART FIFO\n");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
//				LOG_ERR("Drop %u bytes", recv_len - rb_len);
				zprintf("Drop %u bytes\n", recv_len - rb_len);
			}
		}
	}
}
#endif

/**
 * @brief Function for initialize LEDs.
 *
 */
static int init_leds(void) {
    int ret = 0;
    
    for (int i=0; i<NUM_OF_LEDS; i++)
	{
		ret = gpio_pin_configure_dt(&led[i], GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
//			zprintf("gpio configuration error\n");
			return ret;
		}
	}

    gpio_pin_set_dt(&led[0], 0);
    gpio_pin_set_dt(&led[1], 0);
    gpio_pin_set_dt(&led[2], 0);
    gpio_pin_set_dt(&led[3], 0);
    
    return ret;
}

#ifdef DONGLE
/**
 * @brief Function for starting console.
 *
 * @param[in] p_event Pointer to an SAADC driver event.
 */
static int start_console(void) {
    uint32_t baudrate, dtr = 0U;
	int ret;
    
    console_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(console_dev)) {
//		LOG_ERR("CDC ACM device not ready");
		zprintf("CDC ACM device not ready\n");
		return 1;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
//		LOG_ERR("Failed to enable USB");
		zprintf("Failed to enable USB\n");
		return ret;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

//	LOG_INF("Wait for DTR");
	zprintf("Wait for DTR\n");
    while (true) {
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
			break;	//??????????????????
		}
	}

//	LOG_INF("DTR set");
	zprintf("DTR set\n");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(console_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
//		LOG_WRN("Failed to set DCD, ret code %d", ret);
		zprintf("Failed to set DCD, ret code %d\n", ret);
	}

	ret = uart_line_ctrl_set(console_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
//		LOG_WRN("Failed to set DSR, ret code %d", ret);
		zprintf("Failed to set DSR, ret code %d\n", ret);
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(console_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
//		LOG_WRN("Failed to get baudrate, ret code %d", ret);
		zprintf("Failed to get baudrate, ret code %d\n", ret);
	} else {
//		LOG_INF("Baudrate detected: %d", baudrate);
		zprintf("Baudrate detected: %d\n", baudrate);
	}

	uart_irq_callback_set(console_dev, uart_interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(console_dev);
    
    /* Init zprintf */
	zprint_init(console_dev);
    
    return ret;
}
#endif

/**
 * @brief BT connected callback
 *
 * @param[in] 
 */
static void connected(struct bt_conn *conn, u8_t err)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];

	my_connection = conn;

	if (err) 
	{
		zprintf("Connection failed (err %u)\n", err);
		return;
	}
	else if(bt_conn_get_info(conn, &info))
	{
		zprintf("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		zprintf("Connection established!\n");
		zprintf("Connected to: %s						\n", addr);
		zprintf("Connection Interval: %u				\n", info.le.interval);
		zprintf("Slave Latency: %u						\n", info.le.latency);
		zprintf("Connection Supervisory Timeout: %u		\n", info.le.timeout);
	}
}

/**
 * @brief BT disconnected callback
 *
 * @param[in] 
 */
static void disconnected(struct bt_conn *conn, u8_t reason)
{
	zprintf("Disconnected (reason %u)\n", reason);
	my_connection = NULL;
}

/**
 * @brief BT param req callback
 *
 * @param[in] 
 */
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	//If acceptable params, return true, otherwise return false.
	return true; 
}

/**
 * @brief BT param updated callback
 *
 * @param[in] 
 */
static void le_param_updated(struct bt_conn *conn, u16_t interval, u16_t latency, u16_t timeout)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];
	
	if(bt_conn_get_info(conn, &info))
	{
		zprintf("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		zprintf("Connection parameters updated!\n");
		zprintf("Connected to: %s						\n", addr);
		zprintf("New Connection Interval: %u			\n", info.le.interval);
		zprintf("New Slave Latency: %u					\n", info.le.latency);
		zprintf("New Connection Supervisory Timeout: %u	\n", info.le.timeout);
	}
}

/**
 * @brief BT callbacks struct
 *
 */
static struct bt_conn_cb conn_callbacks = 
{
	.connected				= connected,
	.disconnected   		= disconnected,
	.le_param_req			= le_param_req,
	.le_param_updated		= le_param_updated
};

/**
 * @brief BT ready callback
 *
 * @param[in] 
 */
static void bt_ready(int err)
{
	if (err) 
	{
		zprintf("BLE init failed with error code %d\n", err);
		return;
	}

	//Configure connection callbacks
	bt_conn_cb_register(&conn_callbacks);

	//Initalize services
	err = my_service_init();

	if (err) 
	{
		zprintf("Failed to init LBS (err:%d)\n", err);
		return;
	}

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		zprintf("Advertising failed to start (err %d)\n", err);
		return;
	}

	zprintf("Advertising successfully started\n");

	k_sem_give(&ble_init_ok);
}

/**
 * @brief Unrecoverable error 
 *
 */
static void error(void)
{
	gpio_pin_set_dt(&led[0], 0);
	gpio_pin_set_dt(&led[1], 0);
	gpio_pin_set_dt(&led[2], 1);
	while (true) {
		zprintf("Error!\n");
		/* Spin for ever */
		k_sleep(K_MSEC(1000)); //1000ms
	}
}

/**
 * @brief saadc initialization function
 *
 *
 */
static int init_saadc(void) {
    nrfx_err_t status;
    (void)status;

	status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (status != NRFX_SUCCESS) {
		return -1;
    }
#if defined(__ZEPHYR__)
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0);
#endif
    return 0;
}

/**
 * @brief Print out device info
 *
 *
 */
#define VERSION "  Version 0.1 (beta) - "

void vers(void)
{
	char s1[60]=VERSION, s2[]=__DATE__, s3[]=__TIME__;
	strcat(s1, s2);
	strcat(s1, " * ");
	strcat(s1, s3);
	zprintf("%s\n", s1);
}
static void prompt(void) {
	bt_addr_le_t my_addrs[3];
	size_t 	my_addrs_count;
	char dev[BT_ADDR_LE_STR_LEN];

	zprintf("\nDEVICE NAME  :        %s\n", CONFIG_BT_DEVICE_NAME);
	zprintf("MODEL        :        %s\n", CONFIG_BT_DIS_MODEL);
	zprintf("MANUFACTER   :        %s\n", CONFIG_BT_DIS_MANUF);
	zprintf("SERIAL NUMBER:        %s\n", CONFIG_BT_DIS_SERIAL_NUMBER_STR);
	vers();
	bt_id_get(NULL, &my_addrs_count);
	if (my_addrs_count>3) my_addrs_count = 3;
	bt_id_get(my_addrs, &my_addrs_count);
	for (size_t i=0; i<my_addrs_count; i++)
	{
		bt_addr_le_to_str(&my_addrs[i], dev, sizeof(dev));
		zprintf("ADDR (%d)     :        %s, type %u\n",
	       i+1, dev, my_addrs[i].type);
	}
	zprintf("\n");
}

/**
 * @brief main
 *
 *
 */
#define SIZEBUF 8
#define BT_SEND_PERIOD_MS 200
#define LED_BLINK_1_SEC (1000/BT_SEND_PERIOD_MS)
void main(void)
{
	int ret, rb_len, cnt;
    uint8_t buffer[SIZEBUF];

	my_connection = NULL;

    led2blk = 0;
    
	ret = init_leds();
    if (ret == 0) {
        led2blk |= 1;
    }
#ifdef DONGLE
    ret = start_console();
#endif
    if (ret == 0) {
        led2blk = 2;
    }

//    k_sleep(K_MSEC(5000));
    zprintf("\nUSB Console Init Code=%d\n\r", led2blk);
    cnt = 0;
    
    ret = init_saadc();
    if (ret) {
        zprintf("Saadc init failed\n");
        error(); //Catch error
    }

	ret = bt_enable(bt_ready);

	if (ret) 
	{
		zprintf("BLE initialization failed\n");
		error(); //Catch error
	}
		
	ret = dk_buttons_init(button_handler);
	if (ret) {
		zprintf("Failed to initialize buttons (err %d)\n", ret);
		error(); //Catch error
	}
    
    zprintf("********** Br1Ge9se BLE peripheral ADC **********\n");

	/* 	Bluetooth stack should be ready in less than 100 msec. 								\
																							\
		We use this semaphore to wait for bt_enable to call bt_ready before we proceed 		\
		to the main loop. By using the semaphore to block execution we allow the RTOS to 	\
		execute other tasks while we wait. */	
	ret = k_sem_take(&ble_init_ok, K_MSEC(500));

	if (!ret) 
	{
		zprintf("Bluetooth initialized\n");
	} else 
	{
		zprintf("BLE initialization did not complete in time\n");
		error(); //Catch error
	}

	ret = my_service_init();
    
	while(true) {
		k_sleep(K_MSEC(BT_SEND_PERIOD_MS));
        if (++cnt == LED_BLINK_1_SEC) {
            gpio_pin_toggle_dt(&led[led2blk]);
            cnt = 0;
        }

		saadc_notify();

		if (my_connection != NULL) {
			my_service_send(my_connection, (uint8_t *)&saadc_curr_value, sizeof(saadc_curr_value));
			gpio_pin_set_dt(&led[0], 1);
		}
		else {
			gpio_pin_set_dt(&led[0], 0);
		}
        rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
        if (rb_len > 0) {
            buffer[rb_len] = 0;
			if (buffer[0] == '.') {
				prompt();
			}
			else {
				zprintf("%s", buffer);
				if (buffer[0] == '\r')
					zprintf("\n");
			}
        }
	}
}
