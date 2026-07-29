#pragma once

#include "device_imu.h"
#include "device_mcu.h"

#ifdef __cplusplus
extern "C" {
#endif

device_imu_error_type device_imu_open_hid(device_imu_type* device, device_imu_event_callback callback);

device_mcu_error_type device_mcu_open_hid(device_mcu_type* device, device_mcu_event_callback callback);

#ifdef __cplusplus
} // extern "C"
#endif
