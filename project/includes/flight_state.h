/**
 * @file flight_state.h
 * @brief Flight state machine for rocket trajectory tracking
 * 
 * This module implements the state machine that tracks the rocket's
 * flight phases, from idle on the ground through launch, powered flight,
 * apogee, descent, and landing. It uses sensor data to determine state
 * transitions.
 */

#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H


#include <stdint.h>
#include <stdbool.h>
#include "imu.h"

typedef enum {
    FLIGHT_STATE_GROUND_IDLE,     /* Rocket on ground, pre-launch */
    FLIGHT_STATE_POWERED_FLIGHT,  /* Motor burning, accelerating */
    FLIGHT_STATE_UNPOWERED_FLIGHT,/* Coasting to apogee */
    FLIGHT_STATE_DROGUE_DESCENT,  /* Descending under drogue parachute */
    FLIGHT_STATE_MAIN_DESCENT,    /* Descending under main parachute */
    FLIGHT_STATE_LANDED           /* Landed, mission complete */
} FlightState_t;

/* Flight detection thresholds */
#define LAUNCH_ACCELERATION_THRESHOLD  1.25f   /* Acceleration threshold in g */
#define LAUNCH_DETECTION_COUNT         10      /* Number of consecutive readings above threshold */
#define APOGEE_DETECTION_COUNT         50      /* Readings with negative velocity for apogee */
#define LANDING_DETECTION_COUNT        1000    /* Readings with no new minimum altitude for landing */
#define MIN_ALTITUDE_FOR_APOGEE        100.0f  /* Minimum altitude (ft) for apogee detection */
#define MAIN_CHUTE_ALTITUDE            500.0f  /* Altitude (ft) for main chute deployment */
#define MOTOR_BURN_TIME_MS             1000    /* Motor burn time in milliseconds */


#endif