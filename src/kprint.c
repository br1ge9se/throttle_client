/*
 * Copyright (c) 2010, 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Low-level debug output
 *
 * Low-level debugging output. Platform installs a character output routine at
 * init time. If no routine is installed, a nop routine is called.
 */

#include <zephyr/kernel.h>
#include <stdarg.h>
#include <stdio.h>
#include <sys/types.h>
#include <zephyr/drivers/uart.h>
#include "board_conf.h"

#ifdef DONGLE
#define CFG_PBSIZE 128

static const struct device *dev = NULL;

void zprint_init(const struct device *mydev)
{
    dev = mydev;
}

/**
 * @brief Default character output routine
 * @param c Character to swallow
 *
 */

static void zputc(int c)
{
	uart_poll_out(dev, c);
}

void console_putc(char c)
{
    if (dev != NULL)
        zputc(c);
}

static int console_puts(const char *str)
{
	int n = 0;

	while (*str) {
		if (*str == '\n')
			console_putc('\r');

		console_putc(*str);
		str++;
		n++;
	}

	return n;
}

int zprintf(const char *fmt, ...)
{
	va_list args;
	unsigned int i;
	char printbuffer[CFG_PBSIZE];

	va_start(args, fmt);
	i = vsprintf(printbuffer, fmt, args);
	va_end(args);

	console_puts(printbuffer);

	return i;
}
#endif /* DONGLE */