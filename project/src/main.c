#include <stdint.h>

int main(void) {


    /* Initializing Process Modules*/
    IMU_Init();
    DataLogger_Init();
    FlightState_Init();
    Payload_Init();

#ifdef USE_RTOS
    
#else
    /*Bare Metal Infinite Loop */
    uint32_t lastImuTime = 0;
    uint32_t lastFlightTime = 0;
    uint32_t lastLogTime = 0;

    while (1) {
        uint32_t currentTime = HAL_GetTick(); // get tick from RTS

        /* Run IMU sampling at 200Hz */
        if (currentTime - lastImuTime >= 5) {  // 5ms = 200Hz
            IMU_Sample();
            lastImuTime = currentTime;
        }
        
        /* Run flight logic at 100Hz */
        if (currentTime - lastFlightTime >= 10) { // 10ms = 100Hz
            FlightState_Update();
            lastFlightTime = currentTime;
        }
        
        /* Run data logging at 100Hz */
        if (currentTime - lastLogTime >= 10) {
            DataLogger_Process();
            lastLogTime = currentTime;
        }
    }
#endif

}