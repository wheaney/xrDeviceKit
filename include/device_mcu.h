#pragma once
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

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifndef __cplusplus
#include <stdint.h>
#else
#include <cstdint>
#endif

#include "mcu_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

enum device_mcu_error_t {
	DEVICE_MCU_ERROR_NO_ERROR = 0,
	DEVICE_MCU_ERROR_NO_DEVICE = 1,
	DEVICE_MCU_ERROR_NO_HANDLE = 2,
	DEVICE_MCU_ERROR_NO_ACTIVATION = 3,
	DEVICE_MCU_ERROR_WRONG_SIZE = 4,
	DEVICE_MCU_ERROR_UNPLUGGED = 5,
	DEVICE_MCU_ERROR_UNEXPECTED = 6,
	DEVICE_MCU_ERROR_WRONG_HEAD = 7,
	DEVICE_MCU_ERROR_INVALID_LENGTH = 8,
	DEVICE_MCU_ERROR_NOT_INITIALIZED = 9,
	DEVICE_MCU_ERROR_PAYLOAD_FAILED = 10,
	DEVICE_MCU_ERROR_UNKNOWN = 11,
};

enum device_mcu_event_t {
	DEVICE_MCU_EVENT_UNKNOWN = 0,
	DEVICE_MCU_EVENT_SCREEN_ON = 1,
	DEVICE_MCU_EVENT_SCREEN_OFF = 2,
	DEVICE_MCU_EVENT_BRIGHTNESS_UP = 3,
	DEVICE_MCU_EVENT_BRIGHTNESS_DOWN = 4,
	DEVICE_MCU_EVENT_MESSAGE = 5,
	DEVICE_MCU_EVENT_DISPLAY_MODE_2D = 6,
	DEVICE_MCU_EVENT_DISPLAY_MODE_3D = 7,
	DEVICE_MCU_EVENT_BLEND_CYCLE = 8,
	DEVICE_MCU_EVENT_CONTROL_TOGGLE = 9,
	DEVICE_MCU_EVENT_VOLUME_UP = 10,
	DEVICE_MCU_EVENT_VOLUME_DOWN = 11,
};

typedef enum device_mcu_error_t device_mcu_error_type;
typedef enum device_mcu_event_t device_mcu_event_type;
typedef void (*device_mcu_event_callback)(
		uint64_t timestamp,
		device_mcu_event_type event,
		uint8_t brightness,
		const char* msg
);

struct device_mcu_t {
	uint16_t vendor_id;
	uint16_t product_id;

	void* handle;

	bool activated;
	char mcu_app_fw_version [42];
	char dp_fw_version [42];
	char dsp_fw_version [42];

	bool active;
	uint8_t brightness;
	uint8_t disp_mode;
	uint8_t blend_state;
	uint8_t control_mode;

	device_mcu_event_callback callback;

	const struct mcu_protocol* protocol;
};

typedef struct device_mcu_t device_mcu_type;

device_mcu_error_type device_mcu_open(device_mcu_type* device, device_mcu_event_callback callback);

device_mcu_error_type device_mcu_clear(device_mcu_type* device);

device_mcu_error_type device_mcu_read(device_mcu_type* device, int timeout);

device_mcu_error_type device_mcu_close(device_mcu_type* device);

#ifdef __cplusplus
} // extern "C"
#endif
