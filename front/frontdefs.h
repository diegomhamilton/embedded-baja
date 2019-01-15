#ifndef FRONTDEFS_H
#define FRONTDEFS_H
#endif

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