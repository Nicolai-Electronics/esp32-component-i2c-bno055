#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define BNO055_REG_PAGE_ID               0x07
#define BNO055_REG_P0_CHIP_ID            0x00
#define BNO055_REG_P0_ACC_CHIP_ID        0x01
#define BNO055_REG_P0_MAG_CHIP_ID        0x02
#define BNO055_REG_P0_GYRO_CHIP_ID       0x03
#define BNO055_REG_P0_SW_REV_ID_LSB      0x04
#define BNO055_REG_P0_SW_REV_ID_MSB      0x05
#define BNO055_REG_P0_BOOTLOADER_VERSION 0x06
#define BNO055_REG_P0_ACC_DATA_X_LSB     0x08
#define BNO055_REG_P0_ACC_DATA_X_MSB     0x09
#define BNO055_REG_P0_ACC_DATA_Y_LSB     0x0A
#define BNO055_REG_P0_ACC_DATA_Y_MSB     0x0B
#define BNO055_REG_P0_ACC_DATA_Z_LSB     0x0C
#define BNO055_REG_P0_ACC_DATA_Z_MSB     0x0D
#define BNO055_REG_P0_MAG_DATA_X_LSB     0x0E
#define BNO055_REG_P0_MAG_DATA_X_MSB     0x0F
#define BNO055_REG_P0_MAG_DATA_Y_LSB     0x10
#define BNO055_REG_P0_MAG_DATA_Y_MSB     0x11
#define BNO055_REG_P0_MAG_DATA_Z_LSB     0x12
#define BNO055_REG_P0_MAG_DATA_Z_MSB     0x13
#define BNO055_REG_P0_GYRO_DATA_X_LSB    0x14
#define BNO055_REG_P0_GYRO_DATA_X_MSB    0x15
#define BNO055_REG_P0_GYRO_DATA_Y_LSB    0x16
#define BNO055_REG_P0_GYRO_DATA_Y_MSB    0x17
#define BNO055_REG_P0_GYRO_DATA_Z_LSB    0x18
#define BNO055_REG_P0_GYRO_DATA_Z_MSB    0x19
#define BNO055_REG_P0_HEADING_DATA_LSB   0x1A
#define BNO055_REG_P0_HEADING_DATA_MSB   0x1B
#define BNO055_REG_P0_ROLL_DATA_LSB      0x1C
#define BNO055_REG_P0_ROLL_DATA_MSB      0x1D
#define BNO055_REG_P0_EUL_PITCH_LSB      0x1E
#define BNO055_REG_P0_EUL_PITCH_MSB      0x1F
#define BNO055_REG_P0_QUA_DATA_W_LSB     0x20
#define BNO055_REG_P0_QUA_DATA_W_MSB     0x21
#define BNO055_REG_P0_QUA_DATA_X_LSB     0x22
#define BNO055_REG_P0_QUA_DATA_X_MSB     0x23
#define BNO055_REG_P0_QUA_DATA_Y_LSB     0x24
#define BNO055_REG_P0_QUA_DATA_Y_MSB     0x25
#define BNO055_REG_P0_QUA_DATA_Z_LSB     0x26
#define BNO055_REG_P0_QUA_DATA_Z_MSB     0x27
#define BNO055_REG_P0_LIA_DATA_X_LSB     0x28
#define BNO055_REG_P0_LIA_DATA_X_MSB     0x29
#define BNO055_REG_P0_LIA_DATA_Y_LSB     0x2A
#define BNO055_REG_P0_LIA_DATA_Y_MSB     0x2B
#define BNO055_REG_P0_LIA_DATA_Z_LSB     0x2C
#define BNO055_REG_P0_LIA_DATA_Z_MSB     0x2D
#define BNO055_REG_P0_GRV_DATA_X_LSB     0x2E
#define BNO055_REG_P0_GRV_DATA_X_MSB     0x2F
#define BNO055_REG_P0_GRV_DATA_Y_LSB     0x30
#define BNO055_REG_P0_GRV_DATA_Y_MSB     0x31
#define BNO055_REG_P0_GRV_DATA_Z_LSB     0x32
#define BNO055_REG_P0_GRV_DATA_Z_MSB     0x33
#define BNO055_REG_P0_TEMP               0x34
#define BNO055_REG_P0_CALIB_STAT         0x35
#define BNO055_REG_P0_ST_RESULT          0x36
#define BNO055_REG_P0_INT_STA            0x37
#define BNO055_REG_P0_SYS_CLK_STATUS     0x38
#define BNO055_REG_P0_SYS_STATUS         0x39
#define BNO055_REG_P0_SYS_ERR            0x3A
#define BNO055_REG_P0_UNIT_SEL           0x3B
#define BNO055_REG_P0_OPR_MODE           0x3D
#define BNO055_REG_P0_PWR_MODE           0x3E
#define BNO055_REG_P0_SYS_TRIGGER        0x3F
#define BNO055_REG_P0_TEMP_SOURCE        0x40
#define BNO055_REG_P0_AXIS_MAP_CONFIG    0x41
#define BNO055_REG_P0_AXIS_MAP_SIGN      0x42
#define BNO055_REG_P0_ACC_OFFSET_X_LSB   0x55
#define BNO055_REG_P0_ACC_OFFSET_X_MSB   0x56
#define BNO055_REG_P0_ACC_OFFSET_Y_LSB   0x57
#define BNO055_REG_P0_ACC_OFFSET_Y_MSB   0x58
#define BNO055_REG_P0_ACC_OFFSET_Z_LSB   0x59
#define BNO055_REG_P0_ACC_OFFSET_Z_MSB   0x5A
#define BNO055_REG_P0_MAG_OFFSET_X_LSB   0x5B
#define BNO055_REG_P0_MAG_OFFSET_X_MSB   0x5C
#define BNO055_REG_P0_MAG_OFFSET_Y_LSB   0x5D
#define BNO055_REG_P0_MAG_OFFSET_Y_MSB   0x5E
#define BNO055_REG_P0_MAG_OFFSET_Z_LSB   0x5F
#define BNO055_REG_P0_MAG_OFFSET_Z_MSB   0x60
#define BNO055_REG_P0_GYR_OFFSET_X_LSB   0x61
#define BNO055_REG_P0_GYR_OFFSET_X_MSB   0x62
#define BNO055_REG_P0_GYR_OFFSET_Y_LSB   0x63
#define BNO055_REG_P0_GYR_OFFSET_Y_MSB   0x64
#define BNO055_REG_P0_GYR_OFFSET_Z_LSB   0x65
#define BNO055_REG_P0_GYR_OFFSET_Z_MSB   0x66
#define BNO055_REG_P0_ACC_RADIUS_LSB     0x67
#define BNO055_REG_P0_ACC_RADIUS_MSB     0x68
#define BNO055_REG_P0_MAG_RADIUS_LSB     0x69
#define BNO055_REG_P0_MAG_RADIUS_MSB     0x6A
#define BNO055_REG_P1_ACC_CONFIG         0x08
#define BNO055_REG_P1_MAG_CONFIG         0x09
#define BNO055_REG_P1_GYR_CONFIG_0       0x0A
#define BNO055_REG_P1_GYR_CONFIG_1       0x0B
#define BNO055_REG_P1_ACC_SLEEP_CONFIG   0x0C
#define BNO055_REG_P1_GYR_SLEEP_CONFIG   0x0D
#define BNO055_REG_P1_INT_MSK            0x0F
#define BNO055_REG_P1_INT_EN             0x10
#define BNO055_REG_P1_ACC_AM_THRES       0x11
#define BNO055_REG_P1_ACC_INT_SETTINGS   0x12
#define BNO055_REG_P1_ACC_HG_DURATION    0x13
#define BNO055_REG_P1_ACC_HG_THRES       0x14
#define BNO055_REG_P1_ACC_NM_THRES       0x15
#define BNO055_REG_P1_ACC_NM_SET         0x16
#define BNO055_REG_P1_GYR_INT_SETTING    0x17
#define BNO055_REG_P1_GYR_HR_X_SET       0x18
#define BNO055_REG_P1_GYR_DUR_X          0x19
#define BNO055_REG_P1_GYR_HR_Y_SET       0x1A
#define BNO055_REG_P1_GYR_DUR_Y          0x1B
#define BNO055_REG_P1_GYR_HR_Z_SET       0x1C
#define BNO055_REG_P1_GYR_DUR_Z          0x1D
#define BNO055_REG_P1_GYR_AM_THRES       0x1E
#define BNO055_REG_P1_GYR_AM_SET         0x1F
#define BNO055_REG_P1_GYR_UNIQUE_ID      0x5F

#define BNO055_ID     0xA0
#define BNO055_ACC_ID 0xFB
#define BNO055_MAG_ID 0x32
#define BNO055_GYR_ID 0x0F
#define BNO055_ADDR_A 0x28
#define BNO055_ADDR_B 0x29

typedef struct {
    int16_t accel_offset_x; //  x acceleration offset
    int16_t accel_offset_y; //  y acceleration offset
    int16_t accel_offset_z; //  z acceleration offset
    int16_t mag_offset_x;   //  x magnetometer offset
    int16_t mag_offset_y;   //  y magnetometer offset
    int16_t mag_offset_z;   //  z magnetometer offset
    int16_t gyro_offset_x;  //  x gyroscrope offset
    int16_t gyro_offset_y;  //  y gyroscrope offset
    int16_t gyro_offset_z;  //  z gyroscrope offset
    int16_t accel_radius;   //  acceleration radius
    int16_t mag_radius;     //  magnetometer radius
} bno055_offsets_t;

typedef enum {
    BNO055_POWER_MODE_NORMAL   = 0X00,
    BNO055_POWER_MODE_LOWPOWER = 0X01,
    BNO055_POWER_MODE_SUSPEND  = 0X02
} bno055_powermode_t;

typedef enum {
    BNO055_FUSION_FORMAT_WINDOWS = 0X00,
    BNO055_FUSION_FORMAT_ANDROID = 0X01,
} bno055_fusion_data_output_format_t;

typedef enum {
    BNO055_TEMPERATURE_CELSIUS   = 0X00,
    BNO055_TEMPERATURE_FARENHEIT = 0X01,
} bno055_temperature_unit_t;

typedef enum {
    BNO055_EULER_DEGREES = 0X00,
    BNO055_EULER_RADIANS = 0X01,
} bno055_euler_unit_t;

typedef enum {
    BNO055_ANGULAR_RATE_DPS = 0X00,
    BNO055_ANGULAR_RATE_RPS = 0X01,
} bno055_angular_rate_unit_t;

typedef enum {
    BNO055_ACCELERATION_MS2 = 0X00,
    BNO055_ACCELERATION_MG  = 0X01,
} bno055_acceleration_unit_t;

typedef enum {
    BNO055_OPERATION_MODE_CONFIG       = 0X00,
    BNO055_OPERATION_MODE_ACCONLY      = 0X01,
    BNO055_OPERATION_MODE_MAGONLY      = 0X02,
    BNO055_OPERATION_MODE_GYRONLY      = 0X03,
    BNO055_OPERATION_MODE_ACCMAG       = 0X04,
    BNO055_OPERATION_MODE_ACCGYRO      = 0X05,
    BNO055_OPERATION_MODE_MAGGYRO      = 0X06,
    BNO055_OPERATION_MODE_AMG          = 0X07,
    BNO055_OPERATION_MODE_IMUPLUS      = 0X08,
    BNO055_OPERATION_MODE_COMPASS      = 0X09,
    BNO055_OPERATION_MODE_M4G          = 0X0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    BNO055_OPERATION_MODE_NDOF         = 0X0C
} bno055_opmode_t;

typedef enum {
    BNO055_REMAP_CONFIG_P0 = 0x21,
    BNO055_REMAP_CONFIG_P1 = 0x24, // default
    BNO055_REMAP_CONFIG_P2 = 0x24,
    BNO055_REMAP_CONFIG_P3 = 0x21,
    BNO055_REMAP_CONFIG_P4 = 0x24,
    BNO055_REMAP_CONFIG_P5 = 0x21,
    BNO055_REMAP_CONFIG_P6 = 0x21,
    BNO055_REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

typedef enum {
    BNO055_REMAP_SIGN_P0 = 0x04,
    BNO055_REMAP_SIGN_P1 = 0x00, // default
    BNO055_REMAP_SIGN_P2 = 0x06,
    BNO055_REMAP_SIGN_P3 = 0x02,
    BNO055_REMAP_SIGN_P4 = 0x03,
    BNO055_REMAP_SIGN_P5 = 0x01,
    BNO055_REMAP_SIGN_P6 = 0x07,
    BNO055_REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;

typedef struct {
    uint8_t chip_id;   // chip identifier
    uint8_t accel_rev; // acceleration rev
    uint8_t mag_rev;   // magnetometer rev
    uint8_t gyro_rev;  // gyroscrope rev
    uint16_t sw_rev;   // SW rev
    uint8_t bl_rev;    // bootloader rev
} bno055_rev_info_t;

typedef enum {
    BNO055_VECTOR_ACCELEROMETER = BNO055_REG_P0_ACC_DATA_X_LSB,
    BNO055_VECTOR_MAGNETOMETER  = BNO055_REG_P0_MAG_DATA_X_LSB,
    BNO055_VECTOR_GYROSCOPE     = BNO055_REG_P0_GYRO_DATA_X_LSB,
    BNO055_VECTOR_EULER         = BNO055_REG_P0_EUL_PITCH_LSB,
    BNO055_VECTOR_LINEARACCEL   = BNO055_REG_P0_LIA_DATA_X_LSB,
    BNO055_VECTOR_GRAVITY       = BNO055_REG_P0_GRV_DATA_X_LSB
} bno055_vector_type_t;

typedef struct {
    union {
    double v[3];
    struct {
        double x;
        double y;
        double z;
    };
    // Orientation sensors
    struct {
        double roll;    // Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90 degrees <= roll <= 90 degrees
        double pitch;   // Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180 degrees <= pitch <= 180 degrees)
        double heading; // Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359 degrees
    };
    };
} bno055_vector_t;

//typedef void (*bno055_intr_t)(uint8_t, bool); // Interrupt handler type

typedef struct BNO055 {
    int              i2c_bus;
    int              i2c_address;
    int              pin_interrupt;
} BNO055;

esp_err_t bno055_init(BNO055* device, int i2c_bus, int i2c_address, int pin_interrupt, bool reset);
esp_err_t bno055_destroy(BNO055* device);

esp_err_t bno055_check_id(BNO055* device);
esp_err_t bno055_reset(BNO055* device);

esp_err_t bno055_set_mode(BNO055* device, bno055_opmode_t mode);
esp_err_t bno055_get_mode(BNO055* device, bno055_opmode_t* mode);

esp_err_t bno055_set_power_mode(BNO055* device, bno055_powermode_t mode);
//esp_err_t bno055_set_axis_remap(BNO055* device, bno055_axis_remap_config_t remap_code);
//esp_err_t bno055_set_axis_sign(BNO055* device, bno055_axis_remap_sign_t remap_sign);
//esp_err_t bno055_set_ext_crystal_use(BNO055* device, bool use_crystal);
//esp_err_t bno055_set_sensor_offsets(BNO055* device, bno055_offsets_t* sensor_offsets);

esp_err_t bno055_get_rev_info(BNO055* device, bno055_rev_info_t* rev_info);
//esp_err_t bno055_get_system_status(BNO055* device, uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error);
//esp_err_t bno055_get_calibration(BNO055* device, uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
esp_err_t bno055_get_vector(BNO055* device, bno055_vector_type_t vector_type, bno055_vector_t* vector);
//esp_err_t bno055_get_quat(BNO055* device); // fixme
//esp_err_t bno055_get_temperature(BNO055* device, int8_t* temperature);
//esp_err_t bno055_get_sensor_offsets(BNO055* device, bno055_offsets_t* sensor_offsets);
//esp_err_t bno055_get_calibration_state(BNO055* device, bool* calibrated);

esp_err_t bno055_unit_select(BNO055* device, bno055_fusion_data_output_format_t fusion,
                             bno055_temperature_unit_t temperature, bno055_euler_unit_t euler,
                             bno055_angular_rate_unit_t angular, bno055_acceleration_unit_t acceleration);
