#include "xreal_devices.h"

#include "imu_protocol.h"
#include "mcu_protocol.h"
#include "hid_ids.h"
#include "device.h"

#include <stddef.h>
#include <string.h>

#include <hidapi/hidapi.h>

static const imu_protocol* xreal_imu_protocols[NUM_SUPPORTED_PRODUCTS] = {
    &imu_protocol_hid,       // XREAL Air
    &imu_protocol_hid,       // XREAL Air 2
    &imu_protocol_hid,       // XREAL Air 2 Pro
    &imu_protocol_hid,       // XREAL Air 2 Ultra
    &imu_protocol_xreal_one, // XREAL One Pro
    &imu_protocol_xreal_one, // XREAL One Pro
    &imu_protocol_xreal_one, // XREAL One
    &imu_protocol_xreal_one, // XREAL One
    &imu_protocol_xreal_one, // XREAL One S
    &imu_protocol_xreal_one  // XREAL One S
};

static const imu_protocol* xreal_imu_protocol(uint16_t product_id) {
    for (int i = 0; i < NUM_SUPPORTED_PRODUCTS; i++) {
        if (xreal_product_ids[i] == product_id) {
            return xreal_imu_protocols[i];
        }
    }

    return NULL;
}

static const mcu_protocol* xreal_mcu_protocol(uint16_t product_id) {
    return (xreal_mcu_interface_id(product_id) != -1)? &mcu_protocol_hid : NULL;
}

device_imu_error_type device_imu_open_hid(device_imu_type* device, device_imu_event_callback callback) {
    if (!device) {
        return DEVICE_IMU_ERROR_NO_DEVICE;
    }

    memset(device, 0, sizeof(device_imu_type));
    device->vendor_id 	= xreal_vendor_id;
    device->product_id 	= 0;

    if (!device_init()) {
        return DEVICE_IMU_ERROR_NOT_INITIALIZED;
    }

    struct hid_device_info* info = hid_enumerate(
        device->vendor_id,
        device->product_id
    );

    struct hid_device_info* it = info;
    while (it) {
        const imu_protocol* protocol = xreal_imu_protocol(it->product_id);
        // Convert to minimal info for protocol open
        imu_hid_info mini = {
            .product_id = it->product_id,
            .interface_number = it->interface_number,
            .path = it->path,
        };
        if (protocol && protocol->open(device, &mini)) {
            device->product_id = it->product_id;
            device->protocol = protocol;
            break;
        }

        it = it->next;
    }

    hid_free_enumeration(info);

    if (!device->protocol) {
        return DEVICE_IMU_ERROR_NO_HANDLE;
    }

    return device_imu_open(device, callback);
}

device_mcu_error_type device_mcu_open_hid(device_mcu_type* device, device_mcu_event_callback callback) {
    if (!device) {
        return DEVICE_MCU_ERROR_NO_DEVICE;
    }

    memset(device, 0, sizeof(device_mcu_type));
    device->vendor_id 	= xreal_vendor_id;
    device->product_id 	= 0;

    if (!device_init()) {
        return DEVICE_MCU_ERROR_NOT_INITIALIZED;
    }

    struct hid_device_info* info = hid_enumerate(
        device->vendor_id,
        device->product_id
    );

    struct hid_device_info* it = info;
    while (it) {
        const mcu_protocol* protocol = xreal_mcu_protocol(it->product_id);
        mcu_hid_info mini = {
            .product_id = it->product_id,
            .interface_number = it->interface_number,
            .path = it->path,
        };
        if (protocol && protocol->open(device, &mini)) {
            device->product_id = it->product_id;
            device->protocol = protocol;
            break;
        }

        it = it->next;
    }

    hid_free_enumeration(info);

    if (!device->protocol) {
        return DEVICE_MCU_ERROR_NO_HANDLE;
    }

    return device_mcu_open(device, callback);
}
