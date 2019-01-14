#define SERVO_RUN         866
#define SERVO_MID         1320
#define SERVO_CHOKE       1780

typedef enum
{
    IDLE,           // wait
    SLOW_ACQ,       // acquire temperatures and fuel data
    RPM_ACQ,        // calculate speed
    THROTTLE,       // write throttle position (PWM)
    RADIO_TRANSMIT, // send data for box via radio (SPI)
    DEBUG           // send data for debug
} states_t;