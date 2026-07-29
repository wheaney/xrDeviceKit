#pragma once

#ifndef __cplusplus
#include <stdbool.h>
#include <stdint.h>
#else
#include <cstdbool>
#include <cstdint>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct device_mcu_t;

typedef struct mcu_hid_info {
	uint16_t product_id;
	int interface_number;
	const char* path;
} mcu_hid_info;

typedef struct mcu_event {
	uint64_t timestamp;
	uint32_t event;
	uint8_t brightness;
	char msg [64];
} mcu_event;

typedef struct mcu_protocol {
	bool (*open)(struct device_mcu_t* dev, const struct mcu_hid_info* info);
	void (*close)(struct device_mcu_t* dev);

	int (*read_next)(struct device_mcu_t* dev, struct mcu_event* out, int timeout_ms);
} mcu_protocol;

extern const mcu_protocol mcu_protocol_hid;

#ifdef __cplusplus
}
#endif
