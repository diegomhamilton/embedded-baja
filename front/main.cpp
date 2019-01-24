#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "frontdefs.h"
#include "CANMsg.h"
#include "LSM6DS3.h"
#include "Kalman.h"

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
/* I/O pins */
InterruptIn freq_sensor(PB_10);
InterruptIn choke_switch(PA_5, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PA_7, PullUp);       // servomotor RUN mode
/* Debug pins */
DigitalOut led(PC_13);

/* Interrupt services routine */
void canISR();
void servoSwitchISR();
void ticker2HzISR();
void ticker10HzISR();
void ticker20HzISR();
void tickerTrottleISR();
void frequencyCounterISR();
/* Interrupt handlers */
void canHandler();
/* General functions*/
void filterMessage(CANMsg msg);
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt);

/* Debug variables */
Timer t;
bool buffer_full = false;
uint32_t counter = 0;
Timer trottle_tim;
/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker2Hz;
Ticker ticker10Hz;
Ticker ticker20Hz;
Ticker tickerTrottle;
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
state_t current_state = IDLE_ST;
uint8_t pulse_counter = 0;
uint64_t current_period = 0, last_count = 0, last_acq = 0;
float speed = 0, angle_roll = 0, angle_pitch = 0;
int16_t acc_x = 0, acc_y = 0, acc_z = 0, dps_x = 0, dps_y = 0, dps_z = 0;
uint16_t dt = 0;
packet_t data;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    can.attach(&canISR, CAN::RxIrq);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
    ticker2Hz.attach(&ticker2HzISR, 0.1);
    ticker10Hz.attach(&ticker10HzISR, 0.1);
    ticker20Hz.attach(&ticker20HzISR, 0.05);
    LSM6DS3 LSM6DS3(PB_7, PB_6);
    uint16_t lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS,  \
                                      LSM6DS3.A_SCALE_2G,      \
                                      LSM6DS3.G_ODR_26_BW_2,   \
                                      LSM6DS3.A_ODR_26); 
                   
    while (true) {
        if(trottle_tim.read_ms() >= 100)
        {
            trottle_tim.stop();
            trottle_tim.reset();
            choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
            run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
            choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
            run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
        }
        serial.printf("%d\r\n", counter);
        if (state_buffer.full())
        {
            buffer_full = true;
            led = 0;
        }
        else
        {
            led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

        switch (current_state)
        {
            case IDLE_ST:
//                Thread::wait(2);
                break;
            case SLOWACQ_ST:
                break;
            case IMU_ST:
                LSM6DS3.readAccel();                        // read accelerometer data into LSM6DS3.aN_raw
                LSM6DS3.readGyro();                         //  "   gyroscope data into LSM6DS3.gN_raw
                dt = t.read_ms() - last_acq;
                last_acq = t.read_ms();
                acc_x = LSM6DS3.ax_raw;
                acc_y = LSM6DS3.ay_raw;
                acc_z = LSM6DS3.az_raw;
                dps_x = LSM6DS3.gx_raw;
                dps_y = LSM6DS3.gy_raw;
                dps_z = LSM6DS3.gz_raw;
                calcAngles(acc_x, acc_y, acc_z, dps_x, dps_y, dps_z, dt);
                 /* Send accelerometer data */
                txMsg.clear(IMU_ACC_ID);
                txMsg << acc_x << acc_y << acc_z;
                if(can.write(txMsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    txMsg.clear(IMU_DPS_ID);
                    txMsg << dps_x << dps_y << dps_z << dt;
                    can.write(txMsg);
                }
                break;
            case SPEED_ST:
                freq_sensor.fall(NULL);         // disable interrupt
                if (current_period != 0)
                {
                    speed = 1000000*((float)pulse_counter/current_period);    //calculates frequency in Hz
                }
                else
                {
                    speed = 0;
                }
                pulse_counter = 0;                          
                current_period = 0;                         //|-> reset pulses related variables
                last_count = t.read_us();        
                freq_sensor.fall(&frequencyCounterISR);     // enable interrupt
                /* Send speed data */
                txMsg.clear(SPEED_ID);
                txMsg << speed;
                can.write(txMsg);
//                state_buffer.push(DEBUG_ST);                // debug
                break;
            case THROTTLE_ST:
                if (switch_clicked)
                {                    
                    switch_state = !choke_switch.read() << 1 | !run_switch.read() << 0;
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << switch_state;                  // append data (8 bytes max)
                    can.write(txMsg);
                    
                    switch_clicked = false;
                }
                break;
            case DISPLAY_ST:
                break;
            case DEBUG_ST:
                serial.printf("bf=%d, cr=%d\r\n", buffer_full, switch_state);
                serial.printf("roll=%f, pitch=%f\r\n", angle_roll, angle_pitch);
                break;
            default:
                break;
        }
    }
}

/* Interrupt services routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void servoSwitchISR()
{
    counter++;
    trottle_tim.start();
    choke_switch.rise(NULL);     //  throttle interrupt in both edges dettach
    run_switch.rise(NULL);       //  throttle interrupt in both edges dettach
    choke_switch.fall(NULL);     //  throttle interrupt in both edges dettach
    run_switch.fall(NULL);       //  throttle interrupt in both edges dettach
    switch_clicked = true;
    state_buffer.push(THROTTLE_ST);
}

void ticker2HzISR()
{
    state_buffer.push(DISPLAY_ST);
}

void ticker10HzISR()
{
    state_buffer.push(SPEED_ST);
}

void ticker20HzISR()
{
    state_buffer.push(IMU_ST);
}

void frequencyCounterISR()
{
    pulse_counter++;
    current_period += t.read_us() - last_count;
    last_count = t.read_us();        
}

/* Interrupt handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

/* General functions */
void filterMessage(CANMsg msg)
{
//    serial.printf("id: %d\r\n", msg.id);
    
    if (msg.id == THROTTLE_ID)
    {
        switch_clicked = true;
        state_buffer.push(THROTTLE_ST);
        msg >> switch_state;
    }
}

/* Function adapted from Kristian Lauszus library example, source: https://github.com/TKJElectronics/KalmanFilter*/
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt){
    static Kalman kalmanX, kalmanY;
    float kalAngleX, kalAngleY;
    float pitch, roll;
    float gyroXrate, gyroYrate;
    float ax, ay, az;
    static bool first_execution = true;
    
    ax = (float) accx * TO_G;
    ay = (float) accy * TO_G;
    az = (float) accz * TO_G;
    pitch = atan2(ay, az) * RAD_TO_DEGREE;
    roll = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEGREE;
    gyroXrate = grx / TO_DPS;                            // Convert to deg/s
    gyroYrate = gry / TO_DPS;                            // Convert to deg/s

    if (first_execution)
    {
        // set starting angle if first execution
        first_execution = false;
        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);
    }
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    }
    else
    {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if(abs(kalAngleX) > 90)
    {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    angle_roll = kalAngleX;
    angle_pitch = kalAngleY;
}
