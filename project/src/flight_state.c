#include "../includes/flight_state.h"

/* Flight state definitions */
typedef enum {
    FLIGHT_STATE_GROUND_IDLE,
    FLIGHT_STATE_POWERED_FLIGHT,
    FLIGHT_STATE_UNPOWERED_FLIGHT,
    FLIGHT_STATE_DROGUE_DESCENT,
    FLIGHT_STATE_MAIN_DESCENT,
    FLIGHT_STATE_LANDED
} FlightState_t;


/* State variables */
static FlightState_t currentState = FLIGHT_STATE_GROUND_IDLE;
static float groundAltitude = 0.0f;
static uint32_t launchTime = 0;
static uint32_t apogeeTime = 0;
static uint32_t landingTime = 0;
static uint8_t consecutiveReadingsLaunch = 0;
static uint8_t consecutiveReadingsApogee = 0;
static uint16_t consecutiveReadingsLanding = 0;
static float previousAltitude = 0.0f;


/* Init flight state machine */
void FlightState_Init(void) {
    currentState = FLIGHT_STATE_GROUND_IDLE;
    groundAltitude = IMU_CalculateGroundAltitude();
    
}

void FlightState_Update(void) {
    IMU_Data_t imuData = IMU_GetLatestData();

    float altitude = imuData.altitude;

    float filteredAltitude, filteredVelocity;

    KalmanFilter_Update(altitude, &filteredAltitude, &filteredVelocity);

    /*Ground Logic*/
    if (currentState == FLIGHT_STATE_GROUND_IDLE) {
        if (fabsf(imuData.accelX) > LAUNCH_ACCELERATION_THRESHOLD) {
            consecutiveReadingsLaunch++;
        } else {
            consecutiveReadingsLaunch = 0;
        }

        if (consecutiveReadingsLaunch >= LAUNCH_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_POWERED_FLIGHT;
            launchTime = HAL_GetTick(); // or whatever function polls the RTS on the STM32
            // DataLogger_LogEvent("Launch Detected");
        }
    }

    /*Apogee Detection*/
    if (currentState == FLIGHT_STATE_POWERED_FLIGHT || currentState == FLIGHT_STATE_UNPOWERED_FLIGHT) {
        
        /*State Change from powered to unpowered ascent*/
        if (currentState == FLIGHT_STATE_POWERED_FLIGHT && 
            (HAL_GetTick() - launchTime) > MOTOR_BURN_TIME_MS)
            currentState = FLIGHT_STATE_UNPOWERED_FLIGHT;
            //DataLogger_LogEvent("Motor Burnout")
        
        /*Check for Apogee*/
        if (filteredVelocity < 0.0f && altitude > (groundAltitude + MIN_ALTITUDE_FOR_APOGEE)) {
            consecutiveReadingsApogee++;
        } else {
            consecutiveReadingsApogee = 0;
        }

        if (consecutiveReadingsApogee >= APOGEE_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_DROGUE_DESCENT;
            apogeeTime = HAL_GetTick();
            //DataLogger_LogEvent("Apogee Detected");
            previousAltitude = altitude;
        }
    }

    /*Landing Detection*/
    if (currentState == FLIGHT_STATE_DROGUE_DESCENT || currentState == FLIGHT_STATE_MAIN_DESCENT) {

        /*Check for Main Chute Deployment*/
        if (currentState == FLIGHT_STATE_DROGUE_DESCENT && altitude < (groundAltitude + MAIN_CHUTE_ALTITUDE)) {
            currentState = FLIGHT_STATE_MAIN_DESCENT;
            //DataLogger_LogEvent("Main Descent Detected")
        }

        /*Landed State Condition - No New Altitude in (x) cycles*/
        if (altitude < previousAltitude) {
            previousAltitude = altitude;
            consecutiveReadingsLanding = 0;
        } else {
            consecutiveReadingsLanding++;
        }

        if (consecutiveReadingsLanding > LANDING_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_LANDED;
            // landingTime = HAL_GetTick();
            // DataLogger_LogEvent("Landing Detected");

            // Payload Activate();
        }
    }

    DataLogger_LogState(currentState, altitude, filteredAltitude, filteredVelocity);

}

/* Task function if using RTOS */
#ifdef USE_RTOS
void FlightState_Task(void* parameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10); // 100Hz
    
    while (1) {
        /* Precise timing with vTaskDelayUntil */
        vTaskDelayUntil(&lastWakeTime, frequency);
        
        /* Update flight state */
        FlightState_Update();
    }
}

#endif

