#define REARDEFS_H

#define SERVO_RUN         866
#define SERVO_MID         1320
#define SERVO_CHOKE       1780

typedef enum
{
    IDLE_ST,        // wait
    SLOWACQ_ST,     // acquire temperatures and fuel data
    RPM_ST,         // calculate speed
    THROTTLE_ST,    // write throttle position (PWM)
    RADIO_ST,       // send data for box via radio (SPI)
    DEBUG_ST        // send data for debug
} states_t;