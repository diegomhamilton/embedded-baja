#ifndef FRONTDEFS_H
#define FRONTDEFS_H

/* Kalman's filter definitions */
#define PI              3.1416
#define RAD_TO_DEGREE   180.0/PI
#define TO_G            2.0/32768.0
#define TO_DPS          245.0/32768.0

typedef enum
{
    IDLE_ST,        // wait
    SLOWACQ_ST,     // acquire battery level data
    IMU_ST,         // acquire accelerometer and gyroscope data
    SPEED_ST,       // calculate speed
    THROTTLE_ST,    // check throttle position (from user button)
    DISPLAY_ST,     // send data for display over serial port
    DEBUG_ST        // send data for debug
} state_t;

#endif