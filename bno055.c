/**
 * Copyright (c) 2021 Nicolai Electronics
 *
 * SPDX-License-Identifier: MIT
 */

#include <sdkconfig.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include "bno055.h"
#include "managed_i2c.h"

static const char *TAG = "BNO055";

uint8_t current_page = 0xFF;

esp_err_t bno055_select_page(BNO055* device, uint8_t page) {
    if (current_page == page) {
        return ESP_OK;
    }
    esp_err_t res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_PAGE_ID, &page, 1);
    if (res != ESP_OK) {
        return res;
    }
    current_page = page;
    return ESP_OK;
}

esp_err_t bno055_set_mode(BNO055* device, bno055_opmode_t mode) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    uint8_t mode_value = mode;
    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_P0_OPR_MODE, &mode_value, 1);
    if (res != ESP_OK) return res;
    vTaskDelay(30 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bno055_get_mode(BNO055* device, bno055_opmode_t* mode) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    return i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_OPR_MODE, (uint8_t*) mode, 1);;
}

esp_err_t bno055_set_power_mode(BNO055* device, bno055_powermode_t mode) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    uint8_t mode_value = mode;
    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_P0_PWR_MODE, &mode_value, 1);
    if (res != ESP_OK) return res;
    return ESP_OK;
}

esp_err_t bno055_get_rev_info(BNO055* device, bno055_rev_info_t* rev_info) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_CHIP_ID, &rev_info->chip_id, 1);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_ACC_CHIP_ID, &rev_info->accel_rev, 1);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_MAG_CHIP_ID, &rev_info->mag_rev, 1);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_GYRO_CHIP_ID, &rev_info->gyro_rev, 1);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_SW_REV_ID_LSB, (uint8_t*) &rev_info->sw_rev, 2);
    if (res != ESP_OK) return res;
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, BNO055_REG_P0_BOOTLOADER_VERSION, &rev_info->bl_rev, 1);
    return res;
}

esp_err_t bno055_check_id(BNO055* device) {
    esp_err_t res;
    bno055_rev_info_t rev_info;
    res = bno055_get_rev_info(device, &rev_info);
    if (res != ESP_OK) return res;
    if (rev_info.chip_id != BNO055_ID) {
        ESP_LOGE(TAG, "Chip id mismatch");
        return ESP_FAIL;
    }
    if (rev_info.accel_rev != BNO055_ACC_ID) {
        ESP_LOGE(TAG, "Acc id mismatch");
        return ESP_FAIL;
    }
    if (rev_info.mag_rev != BNO055_MAG_ID) {
        ESP_LOGE(TAG, "Mag id mismatch");
        return ESP_FAIL;
    }
    if (rev_info.gyro_rev != BNO055_GYR_ID) {
        ESP_LOGE(TAG, "Gyro id mismatch");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t bno055_reset(BNO055* device) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    uint8_t value = 0x20;
    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_P0_SYS_TRIGGER, &value, 1);
    if (res != ESP_OK) return res;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t timeout = 200;
    bool pass = false;
    while (timeout > 0) {
        res = bno055_check_id(device);
        if (res == ESP_OK) {
            pass = true;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timeout--;
    }
    if (!pass) return ESP_FAIL;
    ESP_LOGI(TAG, "Checking identifiers after reset took %u tries", 200-timeout);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bno055_unit_select(BNO055* device, bno055_fusion_data_output_format_t fusion,
                             bno055_temperature_unit_t temperature, bno055_euler_unit_t euler,
                             bno055_angular_rate_unit_t angular, bno055_acceleration_unit_t acceleration) {
    esp_err_t res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;
    uint8_t value = acceleration | (angular << 1) | (euler << 2) | (temperature << 4) | (fusion << 7);
    res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_P0_UNIT_SEL, &value, 1);
    if (res != ESP_OK) return res;
    return res;
}

esp_err_t bno055_init(BNO055* device, int i2c_bus, int i2c_address, int pin_interrupt, bool reset) {    
    esp_err_t res = ESP_OK;
    
    device->i2c_bus = i2c_bus;
    device->i2c_address = i2c_address;
    device->pin_interrupt = pin_interrupt;
    
    if (reset) {
        res = bno055_set_mode(device, BNO055_OPERATION_MODE_CONFIG);
        if (res != ESP_OK) return res;
        
        res = bno055_reset(device);
        if (res != ESP_OK) return res;
        
        res = bno055_set_power_mode(device, BNO055_POWER_MODE_NORMAL);
        if (res != ESP_OK) return res;
        
        /*res = bno055_unit_select(device,
                                 BNO055_FUSION_FORMAT_WINDOWS,
                                 BNO055_TEMPERATURE_CELSIUS,
                                 BNO055_EULER_DEGREES,
                                 BNO055_ANGULAR_RATE_DPS,
                                 BNO055_ACCELERATION_MS2);
        if (res != ESP_OK) return res;*/
        
        res = bno055_set_mode(device, BNO055_OPERATION_MODE_NDOF);
        if (res != ESP_OK) return res;
        
        uint8_t value = 0x00;
        res = i2c_write_reg_n(device->i2c_bus, device->i2c_address, BNO055_REG_P0_SYS_TRIGGER, &value, 1);
        if (res != ESP_OK) return res;
    }

    return res;
}

esp_err_t bno055_destroy(BNO055* device) {
    return ESP_OK;
}

esp_err_t bno055_get_vector(BNO055* device, bno055_vector_type_t vector_type, bno055_vector_t* vector) {
    esp_err_t res;

    res = bno055_select_page(device, 0);
    if (res != ESP_OK) return res;

    uint8_t buffer[6];
    res = i2c_read_reg(device->i2c_bus, device->i2c_address, vector_type, (uint8_t*) &buffer, 6);
    if (res != ESP_OK) return res;
    
    int16_t x = ((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8);
    int16_t y = ((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8);
    int16_t z = ((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8);
    
    switch (vector_type) {
        case BNO055_VECTOR_MAGNETOMETER:
            // 1uT = 16 LSB
            vector->v[0] = ((double) x) / 16.0;
            vector->v[1] = ((double) y) / 16.0;
            vector->v[2] = ((double) z) / 16.0;
            break;
        case BNO055_VECTOR_GYROSCOPE:
            // 1dps = 16 LSB
            vector->v[0] = ((double) x) / 16.0;
            vector->v[1] = ((double) y) / 16.0;
            vector->v[2] = ((double) z) / 16.0;
            break;
        case BNO055_VECTOR_EULER:
            // 1 degree = 16 LSB
            vector->v[0] = ((double) x) / 16.0;
            vector->v[1] = ((double) y) / 16.0;
            vector->v[2] = ((double) z) / 16.0;
            break;
        case BNO055_VECTOR_ACCELEROMETER:
            // 1m/s^2 = 100 LSB
            vector->v[0] = ((double) x) / 100.0;
            vector->v[1] = ((double) y) / 100.0;
            vector->v[2] = ((double) z) / 100.0;
            break;
        case BNO055_VECTOR_LINEARACCEL:
            // 1m/s^2 = 100 LSB
            vector->v[0] = ((double) x) / 100.0;
            vector->v[1] = ((double) y) / 100.0;
            vector->v[2] = ((double) z) / 100.0;
            break;
        case BNO055_VECTOR_GRAVITY:
            // 1m/s^2 = 100 LSB
            vector->v[0] = ((double) x) / 100.0;
            vector->v[1] = ((double) y) / 100.0;
            vector->v[2] = ((double) z) / 100.0;
            break;
        default:
            return ESP_FAIL;
    }
    
    return ESP_OK;
}
