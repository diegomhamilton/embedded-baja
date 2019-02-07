#ifndef REARDEFS_H
#define REARDEFS_H

#define SERVO_RUN           866
#define RUN_MODE            0x01
#define SERVO_MID           1320
#define MID_MODE            0x00
#define SERVO_CHOKE         1780
#define CHOKE_MODE          0x02
#define VCC                 3.3
#define R_TERM              1000

#define N_IMU               0             // imu packet counter 
#define N_RPM               1             // rpm packet counter
#define N_FLAG              2            // flag packet counter
#define N_SPEED             3           // speed packet counter

/* Radio definitions */
#define NETWORK_ID          101
#define BOXRADIO_ID         69
#define NODE_ID             55
#define FREQUENCY_915MHZ    91

uint8_t packet_counter[4];

typedef enum
{
    IDLE_ST,        // wait
    SLOWACQ_ST,     // acquire temperatures and fuel data
    RPM_ST,         // calculate speed
    THROTTLE_ST,    // write throttle position (PWM)
    RADIO_ST,       // send data for box via radio (SPI)
    DEBUG_ST        // send data for debug
} state_t;

#endif