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
LSM6DS3 LSM6DS3(PB_7, PB_6);

/* I/O pins */
InterruptIn freq_sensor(PB_10, PullNone);
InterruptIn choke_switch(PA_5, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PA_7, PullUp);       // servomotor RUN mode
InterruptIn horn_button(PB_1, PullUp);
InterruptIn headlight_switch(PB_0, PullUp);
DigitalOut horn(PB_11);
DigitalOut headlight(PA_1);
/* Debug pins */
DigitalOut led(PC_13);
DigitalOut dbg1(PC_14);
DigitalOut dbg2(PC_15);
DigitalOut dbg3(PA_4);

/* Interrupt services routine */
void canISR();
void servoSwitchISR();
void ticker2HzISR();
void ticker10HzISR();
void ticker20HzISR();
void tickerTrottleISR();
void frequencyCounterISR();
void hornISR();
void headlightISR();
/* Interrupt handlers */
void canHandler();
void throttleDebounceHandler();
void hornDebounceHandler();
void headlightDebounceHandler();
/* General functions*/
void setupInterrupts();
void filterMessage(CANMsg msg);
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt);
void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, bool comb, \
                    bool b, bool tl, bool fl, int16_t gp, int16_t gr, bool box);

/* Debug variables */
Timer t;
bool buffer_full = false;
unsigned int t0, t1;
/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker2Hz;
Ticker ticker10Hz;
Ticker ticker20Hz;
Ticker tickerTrottle;
Timeout debounce_throttle;
Timeout debounce_horn;
Timeout debounce_headlight;
// Timeout horn_limiter;                       // stop sound of horn after a determined period
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00, flags = 0x00, pulse_counter = 0, temp_motor = 0;
state_t current_state = IDLE_ST;
uint64_t current_period = 0, last_count = 0, last_acq = 0;
uint8_t imu_failed = 0;                         // number of times before a new connection attempt with imu 
float speed_hz = 0, angle_roll = 0, angle_pitch = 0;
uint16_t rpm_hz = 0, speed_display = 0, speed_radio = 0, dt = 0; 
packet_t data;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    horn = horn_button.read();                               // horn OFF
    headlight = headlight_switch.read();                          // headlight OFF
    led = 1;                                // led OFF
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    t0 = t.read_us();
    uint16_t lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26); 
    t1 = t.read_us();
//    serial.printf("%d\r\n", (t1 - t0));
    setupInterrupts();          

    while (true) {
        if (state_buffer.full())
        {
            buffer_full = true;
            led = 0;
            state_buffer.pop(current_state);
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
                t0 = t.read_us();
                dbg1 = !dbg1;
                if (lsm_addr)
                {
                    bool nack = LSM6DS3.readAccel();                        // read accelerometer data into LSM6DS3.aN_raw
                    if (!nack)
                        nack = LSM6DS3.readGyro();                         //  "   gyroscope data into LSM6DS3.gN_raw
                    
                    if (nack)
                    {
                        lsm_addr = 0;
                        LSM6DS3.ax_raw = 0;
                        LSM6DS3.ay_raw = 0;
                        LSM6DS3.az_raw = 0;
                        LSM6DS3.gx_raw = 0;
                        LSM6DS3.gy_raw = 0;
                        LSM6DS3.gz_raw = 0;
                    }
                }
                else if (imu_failed == IMU_TRIES)
                {
                    lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26);                                    
                    t1 = t.read_us();
                    imu_failed = 0;
//                    serial.printf("%d\r\n", (t1 - t0));
                }
                else
                {
                    imu_failed++;
                }
                
                last_acq = t.read_ms();
//                serial.printf("accz = %d\r\n", LSM6DS3.gz_raw);
                calcAngles(LSM6DS3.ax_raw, LSM6DS3.ay_raw, LSM6DS3.ay_raw, LSM6DS3.gx_raw, LSM6DS3.gy_raw, LSM6DS3.gz_raw, dt);
                 /* Send accelerometer data */
                txMsg.clear(IMU_ACC_ID);
                txMsg << LSM6DS3.ax_raw << LSM6DS3.ay_raw << LSM6DS3.az_raw;
                if(can.write(txMsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    txMsg.clear(IMU_DPS_ID);
                    txMsg << LSM6DS3.gx_raw << LSM6DS3.gy_raw << LSM6DS3.gz_raw << dt;
                    can.write(txMsg);
                }
                break;
            case SPEED_ST:
                dbg2 = !dbg2;
                freq_sensor.fall(NULL);         // disable interrupt
                if (current_period != 0)
                {
                    speed_hz = 1000000*((float)pulse_counter/current_period);    //calculates frequency in Hz
                }
                else
                {
                    speed_hz = 0;
                }
                speed_display = ((float)(PI*WHEEL_DIAMETER*speed_hz)/WHEEL_HOLE_NUMBER);    // make conversion hz to km/h
                speed_radio = ((float)((speed_display)/60.0)*65535);
                pulse_counter = 0;                          
                current_period = 0;                         //|-> reset pulses related variables
                last_count = t.read_us();        
                freq_sensor.fall(&frequencyCounterISR);     // enable interrupt
                /* Send speed data */
                txMsg.clear(SPEED_ID);
                txMsg << speed_radio;
                can.write(txMsg);
//                state_buffer.push(DEBUG_ST);                // debug
                break;
            case THROTTLE_ST:
                dbg3 = !dbg3;
                if (switch_clicked)
                {                    
                    switch_state = !choke_switch.read() << 1 | !run_switch.read() << 0;
                    //serial.printf("switch_state = %d\r\n", switch_state);
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << switch_state;                  // append data (8 bytes max)
                    can.write(txMsg);
                    
                    switch_clicked = false;
                }
                break;
            case DISPLAY_ST:
//                serial.printf("rpm_hz=%d\r\n", rpm_hz);
                displayData(speed_display, rpm_hz, temp_motor, (flags & 0x08), false, true, false, angle_pitch, angle_roll, false);
//                state_buffer.push(DEBUG_ST);
                break;
            case DEBUG_ST:
                serial.printf("imu acc x =%d\r\n", LSM6DS3.ax_raw);
                serial.printf("imu acc y =%d\r\n", LSM6DS3.ay_raw);
                serial.printf("imu acc z =%d\r\n", LSM6DS3.az_raw);
                serial.printf("imu dps x =%d\r\n", LSM6DS3.gx_raw);
                serial.printf("imu dps y =%d\r\n", LSM6DS3.gy_raw);
                serial.printf("imu dps z =%d\r\n", LSM6DS3.gz_raw);
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
    choke_switch.rise(NULL);     //  throttle interrupt in both edges dettach
    run_switch.rise(NULL);       //  throttle interrupt in both edges dettach
    choke_switch.fall(NULL);     //  throttle interrupt in both edges dettach
    run_switch.fall(NULL);       //  throttle interrupt in both edges dettach
    switch_clicked = true;
    debounce_throttle.attach(&throttleDebounceHandler, 0.1);
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

void hornISR()
{
    debounce_horn.attach(&hornDebounceHandler, DEBOUNCE_TIME);
}

void headlightISR()
{
    debounce_headlight.attach(&headlightDebounceHandler, DEBOUNCE_TIME);
}

/* Interrupt handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

void throttleDebounceHandler()
{
    state_buffer.push(THROTTLE_ST);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
}

void hornDebounceHandler()
{
    horn = horn_button.read();
}

void headlightDebounceHandler()
{
    headlight = headlight_switch.read();
}

/* General functions */
void setupInterrupts()
{
    can.attach(&canISR, CAN::RxIrq);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
    horn_button.rise(&hornISR);
    horn_button.fall(&hornISR);
    headlight_switch.rise(&headlightISR);
    headlight_switch.fall(&headlightISR);
    ticker2Hz.attach(&ticker2HzISR, 0.5);
    ticker10Hz.attach(&ticker10HzISR, 0.1);
    ticker20Hz.attach(&ticker20HzISR, 0.05);
}

void filterMessage(CANMsg msg)
{
//    serial.printf("id: %d\r\n", msg.id);

    if(msg.id == RPM_ID)
    {
        msg >> rpm_hz;
    }
    
    else if(msg.id == TEMPERATURE_ID)
    {
        msg >> temp_motor;
    }
    
    else if (msg.id == FLAGS_ID)
    {
        msg >> flags;
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

void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, bool comb, \
                    bool b, bool tl, bool fl, int16_t gp, int16_t gr, bool box)
{
    static uint16_t vel2 = -1, Hz2 = -1, temp2 = -1;
    static int16_t gr2 = -1, gp2 = -1;
    static bool box2 = -1;
    static bool comb2 = -1, b2 = -1, tl2 = -1, fl2 = -1;

    char str[180];
    int aux=0;
    strcpy(str,"");
    
    if(box!=box2)
        sprintf(str + strlen(str), "page1.box.val=%d%c%c%c",box,0xff,0xff,0xff);
    if(vel!=vel2)
        sprintf(str + strlen(str), "page1.v.val=%d%c%c%c",vel,0xff,0xff,0xff);
    if(Hz!=Hz2)
    {
        int r = (Hz*6000)/(5000.0);
        sprintf(str + strlen(str),"page1.r.val=%d%c%c%c",r,0xff,0xff,0xff);
    }
    if(comb!=comb2)
    {
        if (!comb)
            sprintf(str + strlen(str),"page1.c.val=%d%c%c%c",25,0xff,0xff,0xff);
        else
            sprintf(str + strlen(str),"page1.c.val=%d%c%c%c",100,0xff,0xff,0xff);
    }
    if(temp!=temp2)
    {
        int tp = (100*temp)/120;
        sprintf(str + strlen(str),"page1.tp.val=%d%c%c%c",tp,0xff,0xff,0xff);
    }        
    if(b!=b2)
        sprintf(str + strlen(str),"page1.b.val=%d%c%c%c",b,0xff,0xff,0xff);
    if(tl!=tl2)
        sprintf(str + strlen(str),"page1.tl.val=%d%c%c%c",tl,0xff,0xff,0xff);
    if(fl!=fl2)
        sprintf(str + strlen(str),"page1.fl.val=%d%c%c%c",fl,0xff,0xff,0xff);
    if(gp!=gp2)
    {
        int gpn = 7;
        if(gp<=-35){gpn=14;}
        else if(gp>=-30&&gp<-25){gpn=13;}
        else if(gp>=-25&&gp<-20){gpn=12;}
        else if(gp>=-20&&gp<-15){gpn=11;}
        else if(gp>=-15&&gp<-10){gpn=10;}
        else if(gp>=-10&&gp<-5){gpn=9;}
        else if(gp>=-5&&gp<0){gpn=8;}
        else if(gp==0){gpn=7;}
        else if(gp>0&&gp<=5){gpn=6;}
        else if(gp>5&&gp<=10){gpn=5;}
        else if(gp>10&&gp<=15){gpn=4;}
        else if(gp>15&&gp<=20){gpn=3;}
        else if(gp>20&&gp<=25){gpn=2;}
        else if(gp>25&&gp<=30){gpn=1;}
        else if(gp>=35){gpn=0;}
        sprintf(str + strlen(str),"page3.gp.val=%d%c%c%c",gpn,0xff,0xff,0xff);
    }
    if(gr!=gr2)
    {
        int grn = 7;
        if(gr<=-35){grn=0;}
        else if(gr>=-30&&gr<-25){grn=1;}
        else if(gr>=-25&&gr<-20){grn=2;}
        else if(gr>=-20&&gr<-15){grn=3;}
        else if(gr>=-15&&gr<-10){grn=4;}
        else if(gr>=-10&&gr<-5){grn=5;}
        else if(gr>=-5&&gr<0){grn=6;}
        else if(gr==0){grn=7;}
        else if(gr>0&&gr<=5){grn=8;}
        else if(gr>5&&gr<=10){grn=9;}
        else if(gr>10&&gr<=15){grn=10;}
        else if(gr>15&&gr<=20){grn=11;}
        else if(gr>20&&gr<=25){grn=12;}
        else if(gr>25&&gr<=30){grn=13;}
        else if(gr>=35){grn=14;}
        sprintf(str + strlen(str),"page3.gr.val=%d%c%c%c",grn,0xff,0xff,0xff);
    }
    
    while(str[aux]!='\0')
    {
        serial.putc(str[aux]);
        aux++;
    }
    box2 = box;
    vel2 = vel;
    Hz2 = Hz;
    temp2 = temp;
    gr2=gr;
    gp2=gp;
    comb2 = comb;
    b2=b;
    tl2=tl;
    fl2=fl;
}