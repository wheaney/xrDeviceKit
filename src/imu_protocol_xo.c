#include "imu_protocol.h"
#include "device_imu.h"

#include <hidapi/hidapi.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include <xreal_one_driver.h>

#define M_PI 3.14159265358979323846
#define RADIANS_TO_DEGREES (180.0 / M_PI)

typedef struct {
    XrealOneHandle* h;
} xo_ctx;

static bool xo_open_impl(device_imu_type* device, const struct imu_hid_info* info) {
    if (!device) return false;
    device->handle = NULL;

    #ifndef NDEBUG
        printf("[xreal_one] Found IMU device pid=0x%x\n", info->product_id);
    #endif

    xo_ctx* ctx = (xo_ctx*)malloc(sizeof(xo_ctx));
    if (!ctx) return false;
    ctx->h = xo_new();
    if (!ctx->h) { free(ctx); return false; }
    device->handle = ctx; // store context in handle for symmetry
    return true;
}

static void xo_close_impl(device_imu_type* device) {
    if (!device || !device->handle) return;
    xo_ctx* ctx = (xo_ctx*)device->handle;
    if (ctx->h) xo_free(ctx->h);
    free(ctx);
    device->handle = NULL;
}

static bool xo_start_stream(device_imu_type* dev) {
    if (!dev || !dev->handle) return false;
    return true;
}

static bool xo_stop_stream(device_imu_type* dev) {
    if (!dev || !dev->handle) return false;
    return true;
}

static bool xo_get_static_id(device_imu_type* dev, uint32_t* out_id) {
    if (!dev || !dev->handle) return false;
    if (out_id) *out_id = 0;
    return true;
}

static bool xo_load_calibration_json(device_imu_type* dev, uint32_t* len, char** data) {
    if (!dev || !dev->handle) return false;
    if (len) *len = 0;
    if (data) *data = NULL;
    return false; // not supported
}

static uint64_t start_timestamp_ns = 0;
static bool time_debug = false;
static int xo_next_sample(device_imu_type* device, struct imu_sample* out, int timeout_ms) {
    (void)timeout_ms;
    if (!device || !out) return -1;
    xo_ctx* ctx = (xo_ctx*)device->handle;
    if (!ctx || !ctx->h) return -1;

    XOImu imu = (XOImu){0};
    int rc = xo_next(ctx->h, &imu);
    if (rc != 0) return rc; // propagate non-zero (e.g., error or no-sample)

    memset(out, 0, sizeof(*out));
    out->gx = imu.gyro[0] * RADIANS_TO_DEGREES;
    out->gy = imu.gyro[1] * RADIANS_TO_DEGREES;
    out->gz = imu.gyro[2] * RADIANS_TO_DEGREES;
    out->ax = imu.accel[0];
    out->ay = imu.accel[1];
    out->az = imu.accel[2];
    out->mx = out->my = out->mz = NAN; // XO protocol doesn't provide mag
    out->temperature_c = NAN;

    // avoid using IMU timestamp, if possible, as it's apparently inconsistent
    // struct timespec ts;
    // if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    //     const uint64_t ts_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    //     if (start_timestamp_ns != 0) {
    //         out->timestamp_ns = ts_ns - start_timestamp_ns;
    //     } else {
    //         start_timestamp_ns = ts_ns;
    //         out->timestamp_ns = 0;
    //     }
    //     if (!time_debug) {
    //         printf("[xreal_one] Using system time for IMU timestamps\n");
    //     }
    // } else {
        out->timestamp_ns = imu.timestamp * 1000;

    //     if (!time_debug) {
    //         printf("[xreal_one] Using IMU time for IMU timestamps\n");
    //     }
    // }
    time_debug = true;
    out->flags = 0;
    return 1;
}

const imu_protocol imu_protocol_xreal_one = {
    .open = xo_open_impl,
    .close = xo_close_impl,
    .start_stream = xo_start_stream,
    .stop_stream = xo_stop_stream,
    .get_static_id = xo_get_static_id,
    .load_calibration_json = xo_load_calibration_json,
    .next_sample = xo_next_sample,
};
