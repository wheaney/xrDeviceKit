#pragma once
// Abstraction for IMU transport protocols (HID and XREAL ONE)

#ifndef __cplusplus
#include <stdbool.h>
#include <stdint.h>
#else
#include <cstdbool>
#include <cstdint>
#endif

typedef struct imu_hid_info {
    uint16_t product_id;
    int interface_number;
    const char* path;
} imu_hid_info;

#ifdef __cplusplus
extern "C" {
#endif

struct device_imu_t;
struct device_imu_packet_t;

typedef struct imu_sample {
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    float temperature_c;
    uint64_t timestamp_ns;
    uint32_t flags;
} imu_sample;

typedef struct imu_protocol {
    bool (*open)(struct device_imu_t* dev, const struct imu_hid_info* info);
    void (*close)(struct device_imu_t* dev);

    bool (*start_stream)(struct device_imu_t* dev);
    bool (*stop_stream)(struct device_imu_t* dev);

    bool (*get_static_id)(struct device_imu_t* dev, uint32_t* out_id);

    bool (*load_calibration_json)(struct device_imu_t* dev, uint32_t* len, char** data);

    int (*next_sample)(struct device_imu_t* dev, struct imu_sample* out, int timeout_ms);
} imu_protocol;

extern const imu_protocol imu_protocol_hid;
extern const imu_protocol imu_protocol_xreal_one;

#ifdef __cplusplus
}
#endif
