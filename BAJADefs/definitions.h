#ifndef MBED_H
    #include "mbed.h"
    #define MBED_H
#endif

#define BUFFER_SIZE

typedef struct
{
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
} imu_t;
    
typedef struct
{
    uint16_t rpm;
    uint16_t vel;
    uint8_t flags;      // MSB - BOX | SERVO_ERROR | NC | NC | NC | NC | CHK | RUN - LSB
} acq_10hz_t;
    
typedef struct
{  
    uint8_t motor;
    uint8_t cvt;
} temperature_t;
    
typedef struct
{  
    imu_t imu[4];
    acq_10hz_t data_10hz[2];
    temperature_t temp;
    uint8_t data_saved;
} packet_t;

extern CircularBuffer <uint8_t, BUFFER_SIZE> state_buffer;