typedef enum
{
    IDLE,           // wait
    SLOW_ACQ,       // acquire battery level data
    IMU_ACQ,        // acquire accelerometer and gyroscope data
    SPEED_ACQ,      // calculate speed
    THROTTLE,       // check throttle position (from user button)
    DISPLAY,        // send data for display over serial port
    DEBUG           // send data for debug
} states_t;