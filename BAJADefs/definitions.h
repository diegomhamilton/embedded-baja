#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#endif
#ifndef MBED_H
    #include "mbed.h"
    #define MBED_H
#endif

#define CAN_IER         (*((volatile unsigned long *)0x40006414))

#define BUFFER_SIZE     50
#define THROTTLE_MID    0x00
#define THROTTLE_RUN    0x01
#define THROTTLE_CHOKE  0x02
#define THROTTLE_ID     0x100

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