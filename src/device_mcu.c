//
// Created by thejackimonster on 29.03.23.
//
// Copyright (c) 2023-2024 thejackimonster. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "device_mcu.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef NDEBUG
#define device_mcu_error(msg) fprintf(stderr, "ERROR: %s\n", msg)
#else
#define device_mcu_error(msg) (0)
#endif

static void device_mcu_callback(device_mcu_type* device,
							 uint64_t timestamp,
							 device_mcu_event_type event,
							 uint8_t brightness,
							 const char* msg) {
	if (!device->callback) {
		return;
	}

	device->callback(timestamp, event, brightness, msg);
}

device_mcu_error_type device_mcu_open(device_mcu_type* device, device_mcu_event_callback callback) {
	if (!device) {
		device_mcu_error("No device");
		return DEVICE_MCU_ERROR_NO_DEVICE;
	}

	if (!device->protocol) {
		device_mcu_error("No handle");
		return DEVICE_MCU_ERROR_NO_HANDLE;
	}

	device->callback = callback;

	device_mcu_clear(device);

	return DEVICE_MCU_ERROR_NO_ERROR;
}

device_mcu_error_type device_mcu_clear(device_mcu_type* device) {
	return device_mcu_read(device, 10);
}

device_mcu_error_type device_mcu_read(device_mcu_type* device, int timeout) {
	if (!device) {
		device_mcu_error("No device");
		return DEVICE_MCU_ERROR_NO_DEVICE;
	}

	if (!device->handle) {
		device_mcu_error("No handle");
		return DEVICE_MCU_ERROR_NO_HANDLE;
	}

	if (!device->protocol) {
		device_mcu_error("No handle");
		return DEVICE_MCU_ERROR_NO_HANDLE;
	}

	mcu_event event;
	memset(&event, 0, sizeof(event));

	int n = device->protocol->read_next(device, &event, timeout);
	if (n < 0) {
		return DEVICE_MCU_ERROR_UNPLUGGED;
	}

	if (n == 0) {
		return DEVICE_MCU_ERROR_NO_ERROR;
	}

	device_mcu_callback(
			device,
			event.timestamp,
			(device_mcu_event_type) event.event,
			event.brightness,
			(event.msg[0] != '\0')? event.msg : NULL
	);

	return DEVICE_MCU_ERROR_NO_ERROR;
}

device_mcu_error_type device_mcu_close(device_mcu_type* device) {
	if (!device) {
		device_mcu_error("No device");
		return DEVICE_MCU_ERROR_NO_DEVICE;
	}

	if (device->protocol) {
		device->protocol->close(device);
	}

	memset(device, 0, sizeof(device_mcu_type));

	return DEVICE_MCU_ERROR_NO_ERROR;
}
