/**
 * @file imu.h
 * @brief Interface with VectorNav VN-100 IMU sensor for STM32
 * 
 * This module provides functions to initialize, read and parse data from
 * the VectorNav VN-100 IMU sensor on an STM32 microcontroller.
 * It handles UART communication and data interpretation,
 * providing structured sensor readings.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"  /* Replace with your specific STM32 HAL header */

/* IMU configuration constants */
#define IMU_SAMPLE_RATE 200  // Hz
#define IMU_BAUDRATE 115200
#define IMU_TIMEOUT 1000     // milliseconds

/* Data structure to hold IMU readings */
typedef struct {
    float yaw;       /* degrees */
    float pitch;     /* degrees */
    float roll;      /* degrees */
    float a_x;       /* G */
    float a_y;       /* G */
    float a_z;       /* G */
    float pressure;  /* kPa */
    float altitude;  /* feet (derived from pressure) */
    uint32_t timestamp; /* milliseconds since boot */
} IMUData_t;

/**
 * @brief Initialize the IMU
 * 
 * Configures the UART peripheral and sets up the IMU for binary output
 * at the specified sample rate.
 * 
 * @param huart Pointer to the UART handle to use for communication with IMU
 * @return true if initialization was successful, false otherwise
 */
bool IMU_Init(UART_HandleTypeDef *huart);

/**
 * @brief Sample the IMU
 * 
 * Reads the latest data from the IMU, parses it, and stores it in the internal buffer.
 * Should be called at the desired sampling rate (IMU_SAMPLE_RATE).
 * 
 * @return true if new data was successfully read, false otherwise
 */
bool IMU_Sample(void);

/**
 * @brief Get the latest IMU data
 * 
 * @param data Pointer to IMUData_t structure to fill with the latest readings
 * @return true if data was filled with valid readings, false otherwise
 */
bool IMU_GetLatestData(IMUData_t *data);

/**
 * @brief Calculate ground altitude reference
 * 
 * Takes multiple pressure readings and averages them to establish
 * a reference ground altitude.
 * 
 * @return Ground altitude in feet
 */
float IMU_CalculateGroundAltitude(void);

/**
 * @brief Deinitialize the IMU
 * 
 * Resets IMU-related resources.
 */
void IMU_Deinit(void);

#endif /* IMU_H */