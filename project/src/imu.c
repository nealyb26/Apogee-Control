/**
 * @file imu.c
 * @brief Implementation of VectorNav VN-100 IMU interface for STM32
 */

#include "../includes/imu.h"
#include <string.h>
#include <math.h>

/* Private defines */
#define IMU_SYNC_BYTE 0xFA
#define IMU_MESSAGE_LENGTH 35
#define IMU_GROUND_SAMPLES 50
#define SEA_LEVEL_PRESSURE 101.325f /* kPa */
#define IMU_RX_BUFFER_SIZE 128

/* Private variables */
static UART_HandleTypeDef *imuUart = NULL;
static IMUData_t currentData;
static bool dataValid = false;
static uint32_t lastTimestamp = 0;
static uint8_t rxBuffer[IMU_RX_BUFFER_SIZE];
static volatile uint8_t rxIndex = 0;
static volatile bool messageReady = false;

/* Private function prototypes */
static uint16_t IMU_CalculateCRC(const uint8_t *data, size_t length);
static bool IMU_ParseMessage(const uint8_t *message, IMUData_t *data);
static bool IMU_SendCommand(const char *command);
static bool IMU_WaitForMessage(uint8_t *buffer, size_t bufferSize, uint32_t timeout);
static float IMU_CalculateAltitude(float pressure);
static void IMU_UART_RxCallback(void);

/* Public function implementations */

bool IMU_Init(UART_HandleTypeDef *huart) {
    /* Store UART handle */
    if (huart == NULL) {
        return false;
    }
    imuUart = huart;
    
    /* Configure UART for IMU */
    imuUart->Init.BaudRate = IMU_BAUDRATE;
    imuUart->Init.WordLength = UART_WORDLENGTH_8B;
    imuUart->Init.StopBits = UART_STOPBITS_1;
    imuUart->Init.Parity = UART_PARITY_NONE;
    imuUart->Init.Mode = UART_MODE_TX_RX;
    imuUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    imuUart->Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(imuUart) != HAL_OK) {
        return false;
    }
    
    /* Setup interrupt-based reception */
    HAL_UART_Receive_IT(imuUart, &rxBuffer[rxIndex], 1);
    
    /* Initialize IMU with binary output configuration */
    /*
     * $VNWRG - write register
     * 75 - register 75 is "binary output message configuration"
     * 2 - message sent out on port 2 (3V logic levels)
     * 4 - rate divisor (800 / 4 = 200 Hz)
     * 05 - selecting groups 1 and 3
     * 0108 - selecting YPR and Acceleration from group 1
     * 0020 - selecting pressure from group 3
     */
    HAL_Delay(2000);  /* 2 second delay to stabilize connection */
    
    if (!IMU_SendCommand("$VNWRG,75,2,4,05,0108,0020*XX\r\n")) {
        return false;
    }
    
    HAL_Delay(1000);  /* 1 second delay to allow IMU to process command */
    
    /* Initialize data structures */
    memset(&currentData, 0, sizeof(currentData));
    dataValid = false;
    lastTimestamp = 0;
    
    return true;
}

bool IMU_Sample(void) {
    uint8_t buffer[IMU_MESSAGE_LENGTH];
    
    if (imuUart == NULL) {
        return false;
    }
    
    /* Check if we have a new message */
    if (!messageReady) {
        return false;
    }
    
    /* Lock to prevent changes to rxBuffer while copying */
    __disable_irq();
    
    /* Find sync byte in received data */
    uint8_t syncIndex = 0xFF;
    for (uint8_t i = 0; i < rxIndex; i++) {
        if (rxBuffer[i] == IMU_SYNC_BYTE) {
            syncIndex = i;
            break;
        }
    }
    
    if (syncIndex == 0xFF || syncIndex + IMU_MESSAGE_LENGTH > rxIndex) {
        /* No sync byte found or message incomplete */
        messageReady = false;
        rxIndex = 0;
        __enable_irq();
        return false;
    }
    
    /* Copy message data (excluding sync byte which is checked separately) */
    memcpy(buffer, &rxBuffer[syncIndex + 1], IMU_MESSAGE_LENGTH);
    
    /* Reset buffer state */
    messageReady = false;
    rxIndex = 0;
    __enable_irq();
    
    /* Restart receive in interrupt mode */
    HAL_UART_Receive_IT(imuUart, &rxBuffer[rxIndex], 1);
    
    /* Validate CRC */
    uint16_t calc_crc = IMU_CalculateCRC(buffer, 33);
    uint16_t recv_crc = (buffer[34] << 8) | buffer[33];
    
    if (calc_crc != recv_crc) {
        return false;
    }
    
    /* Parse message into data structure */
    IMUData_t newData;
    if (!IMU_ParseMessage(buffer, &newData)) {
        return false;
    }
    
    /* Update current data */
    memcpy(&currentData, &newData, sizeof(IMUData_t));
    currentData.timestamp = HAL_GetTick();
    dataValid = true;
    lastTimestamp = currentData.timestamp;
    
    return true;
}

bool IMU_GetLatestData(IMUData_t *data) {
    if (!dataValid || imuUart == NULL) {
        return false;
    }
    
    memcpy(data, &currentData, sizeof(IMUData_t));
    return true;
}

float IMU_CalculateGroundAltitude(void) {
    float altitude_readings[IMU_GROUND_SAMPLES];
    int valid_readings = 0;
    IMUData_t data;
    
    /* Collect multiple samples */
    for (int i = 0; i < IMU_GROUND_SAMPLES; i++) {
        int retry_count = 0;
        const int max_retries = 10;
        
        /* Try to get a valid sample */
        while (retry_count < max_retries) {
            if (IMU_Sample() && IMU_GetLatestData(&data)) {
                altitude_readings[valid_readings++] = data.altitude;
                break;
            }
            HAL_Delay(10);  /* Wait 10ms before retry */
            retry_count++;
        }
        
        HAL_Delay(10);  /* Wait 10ms between samples */
    }
    
    if (valid_readings == 0) {
        return 0.0f;
    }
    
    /* Calculate average */
    float sum = 0.0f;
    for (int i = 0; i < valid_readings; i++) {
        sum += altitude_readings[i];
    }
    
    float groundAltitude = sum / valid_readings;
    return groundAltitude;
}

void IMU_Deinit(void) {
    imuUart = NULL;
    dataValid = false;
}

/* Private function implementations */

static uint16_t IMU_CalculateCRC(const uint8_t *data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (crc >> 8) | ((crc << 8) & 0xFFFF);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= (crc << 12) & 0xFFFF;
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF;
    }
    return crc;
}

static bool IMU_ParseMessage(const uint8_t *message, IMUData_t *data) {
    if (!message || !data) {
        return false;
    }
    
    /* Extract YPR (degrees) */
    float yaw, pitch, roll;
    memcpy(&yaw, message + 5, 4);
    memcpy(&pitch, message + 9, 4);
    memcpy(&roll, message + 13, 4);
    data->yaw = yaw;
    data->pitch = pitch;
    data->roll = roll;
    
    /* Extract acceleration (convert from m/s² to G) */
    float accel_x, accel_y, accel_z;
    memcpy(&accel_x, message + 17, 4);
    memcpy(&accel_y, message + 21, 4);
    memcpy(&accel_z, message + 25, 4);
    
    const float G = 9.80665f;  /* Earth's gravitational acceleration in m/s² */
    data->a_x = accel_x / G;
    data->a_y = accel_y / G;
    data->a_z = accel_z / G;
    
    /* Extract pressure (kPa) */
    float pressure;
    memcpy(&pressure, message + 29, 4);
    data->pressure = pressure;
    
    /* Calculate altitude */
    data->altitude = IMU_CalculateAltitude(pressure);
    
    return true;
}

static bool IMU_SendCommand(const char *command) {
    if (imuUart == NULL || !command) {
        return false;
    }
    
    size_t len = strlen(command);
    HAL_StatusTypeDef status = HAL_UART_Transmit(imuUart, (uint8_t*)command, len, IMU_TIMEOUT);
    
    return status == HAL_OK;
}

static bool IMU_WaitForMessage(uint8_t *buffer, size_t bufferSize, uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    
    /* Wait for complete message or timeout */
    while (!messageReady && (HAL_GetTick() - startTime < timeout)) {
        /* Just wait */
    }
    
    if (!messageReady) {
        return false;
    }
    
    /* Copy message and reset ready flag */
    __disable_irq();
    if (bufferSize >= rxIndex) {
        memcpy(buffer, rxBuffer, rxIndex);
    }
    messageReady = false;
    __enable_irq();
    
    return true;
}

static float IMU_CalculateAltitude(float pressure) {
    if (pressure <= 0.0f) {
        return 0.0f;  /* Using 0 instead of NAN for embedded systems */
    }
    
    /* Standard altitude equation: h = 145366.45 * (1 - (P/P_0)^0.190284) */
    return 145366.45f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.190284f));
}

/* This function should be called from HAL UART RX complete callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == imuUart) {
        /* Store the received byte */
        rxIndex++;
        
        /* Check if buffer is full */
        if (rxIndex >= IMU_RX_BUFFER_SIZE) {
            rxIndex = 0;
        } else {
            /* Continue receiving in interrupt mode */
            HAL_UART_Receive_IT(imuUart, &rxBuffer[rxIndex], 1);
        }
        
        /* Check if we have enough bytes for a complete message */
        if (rxIndex >= IMU_MESSAGE_LENGTH + 1) {  /* +1 for sync byte */
            messageReady = true;
        }
    }
}