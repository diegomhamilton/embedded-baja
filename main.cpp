#include "mbed.h"
#include "structures.h"
#include "CANMsg.h"
#include "CANDef.h"
#include "LSM6DS3.h"
#include "Kalman.h"

/* Communication protocols */
CAN can(PB_8, PB_9);
Serial uart(PA_9, PA_10);
// I2C i2c(PB_7, PB_6);

/* Debug section */
uint64_t    time_CAN = 0,
            time_CAN2 = 0;
uint32_t calls_CAN = 0;
PwmOut signal_wave(PA_8);           // simulates frequency input
//DigitalOut debug2(PB_15);
//DigitalOut debug3(PB_14);         //PB13, PB12
DigitalOut led(PC_13);

/* RTOS tools declaration */
Thread CANRX_thread(osPriorityNormal1, 1000); //initialize random priority and reserved memory to avoid crashes

/* Pins declaration */
InterruptIn choke(PA_5, PullUp);                                // servomotor CHOKE mode
InterruptIn run(PA_6, PullUp);                                  // servomotor RUN mode
InterruptIn freq_sensor(PB_4);                                  // frequency sensor (rpm, speed)
// AnalogIn    temperature();

/* Timers */
Ticker tick_10hz;                                               // timer for 10hz acquisitions
Ticker tick_25hz;                                              // timer for 25hz acquisitions
Ticker tick_serial;                                             // timer for serial send 
//Timer pulse_timer;

/* Global variables */
Packet* data = new Packet();
uint16_t pulse_counter = 0;
uint16_t current_period = 0;
uint32_t last_count = Kernel::get_ms_count();                   // store moment of last interrupt (PB_4)
CircularBuffer <state_t, STATE_BUF_SIZE> state_buffer;          // store sequence of events to proccess
CircularBuffer <packet_25hz_t, ACQ_BUF_SIZE> acq25hz_buffer;  // store packets of accelerometer and gyro

/* Function declarations */
void freq_ISR();
void choke_ISR();
void run_ISR();
void tick_10hz_ISR();
void tick_25hz_ISR();
void tick_serial_ISR();
void CANRX_check();
uint16_t filter_msg(CANMsg& msg);
void gyro_getdata(uint16_t accx, uint16_t accy, uint16_t accz, uint16_t grx, uint16_t gry, uint16_t grz, uint16_t dt);

void main()
{
    /* Variables declaration */
    state_t st;
    uint16_t frequency;
    uint64_t last_acq = 0;
    CANMsg txmsg;
    LSM6DS3 LSM6DS3(PB_7, PB_6);
    /* RTOS set up */
    osThreadId id = osThreadGetId();                        //get id from main thread
    osThreadSetPriority(osThreadGetId(), osPriorityNormal); // set priority of main thread 1 level before CANRX_thread
    /* Initialization of peripherals (communication, tickers, interrupts) */
    uart.baud(9600);                                        // set UART baud to 9600 (change do 115200)
    can.frequency(5000000);                                 // set CAN rate to 1Mbps
    signal_wave.period(0.02f);                              // set frequency to 50hz
    signal_wave.write(0.5f);                                // set 50% duty cycle
    LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS,   \
                   LSM6DS3.A_SCALE_2G,      \
                   LSM6DS3.G_ODR_26_BW_2,       \
                   LSM6DS3.A_ODR_26);                       // set 245DPS gyro scale, set 2G acc scale, set sampling frequency to 104Hz
    tick_10hz.attach(tick_10hz_ISR, 0.1);                   // configure FREQ state ticker for 10Hz
    tick_25hz.attach(tick_25hz_ISR, 0.04);                 // configure FREQ state ticker for 10Hz
    tick_serial.attach(tick_serial_ISR, 0.5);               // configure SERIAL state ticker for 2Hz
    freq_sensor.fall(freq_ISR);                             // |
    choke.fall(choke_ISR);                                  // |->attach interrupt callback
    run.fall(run_ISR);                                      // |
    
    CANRX_thread.start(CANRX_check);

    while(true)
    {
        if (state_buffer.full())
            uart.printf("buffer full");
        if(!state_buffer.empty())
            state_buffer.pop(st);
        else
            st = IDLE;
        
        switch(st)
        {
            case ACQ:
                LSM6DS3.readAccel();                        // read accelerometer data into LSM6DS3.aN_raw
                LSM6DS3.readGyro();                         //  "   gyroscope data into LSM6DS3.gN_raw
                data->sample_25hz.dt = Kernel::get_ms_count() - last_acq;
                last_acq = Kernel::get_ms_count();
                data->sample_25hz.accx = LSM6DS3.ax_raw;
                data->sample_25hz.accy = LSM6DS3.ay_raw;
                data->sample_25hz.accz = LSM6DS3.az_raw;
                data->sample_25hz.dpsx = LSM6DS3.gx_raw;
                data->sample_25hz.dpsy = LSM6DS3.gy_raw;
                data->sample_25hz.dpsz = LSM6DS3.gz_raw;
                gyro_getdata(data->sample_25hz.accx, data->sample_25hz.accy, data->sample_25hz.accz, \
                             data->sample_25hz.dpsx, data->sample_25hz.dpsy, data->sample_25hz.dpsz, \
                             data->sample_25hz.dt);
                /* Send accelerometer data */
                txmsg.clear();
                txmsg.id = ACC_Msg;
                txmsg << data->sample_25hz.accx << data->sample_25hz.accy << data->sample_25hz.accz;
                if(can.write(txmsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    txmsg.clear();
                    txmsg.id = DPS_Msg;
                    txmsg << data->sample_25hz.dpsx << data->sample_25hz.dpsy << data->sample_25hz.dpsz << data->sample_25hz.dt;
                    can.write(txmsg);
                }
                break;
            case CHKRUN:
                data->chk_run = !choke.read() << 1 | !run.read() << 0;
                txmsg.clear();
                txmsg.id = FLAGS_Msg;
                txmsg << data->chk_run;
                can.write(txmsg);
                break;
            case FREQ:
                freq_sensor.fall(NULL);                     // disable interrupt
                data->sample_10hz.speed = 1000*((float)pulse_counter/current_period);    //calculates frequency in Hz
                // makes conversion to speed
                pulse_counter = 0;                          //|
                current_period = 0;                         //|-> reset pulses related variables
                last_count = Kernel::get_ms_count();        //|
                freq_sensor.fall(freq_ISR);                 // enable interrupt
                /* Send frequency */
                txmsg.clear();
                txmsg.id = SPD_Msg;
                txmsg << data->sample_10hz.speed << (uint16_t) 0;
                can.write(txmsg);
                break;
            case SERIAL:
                // formats string to send to display or radio
                uart.printf("speed: %d\r\n", data->sample_10hz.speed);
                uart.printf("dpsx = %d\r\n", data->sample_25hz.dpsx);
                uart.printf("CR %d\r\n", data->chk_run);    // debug
                uart.printf("calls = %d\r\n", calls_CAN);
                calls_CAN = 0;
                break;
            default:
                uart.printf("*\r\n");                       // debug
                Thread::wait(2);
                break;
        }
    }
}

void freq_ISR()
{
    pulse_counter++;
    current_period += Kernel::get_ms_count() - last_count;  // sum passed time since last interrupt
    last_count = Kernel::get_ms_count();
}

void choke_ISR()
{   
    state_buffer.push(CHKRUN);
}

void run_ISR()
{
    state_buffer.push(CHKRUN);
}

void tick_10hz_ISR()
{
    state_buffer.push(FREQ);
}

void tick_25hz_ISR()
{
    state_buffer.push(ACQ);
}

void tick_serial_ISR()
{
    state_buffer.push(SERIAL);
}

void CANRX_check()
{
    CANMsg      rxmsg;
    uint16_t    wait_time = 10,
                timeout = 0;

    while(true)
    {
        if(can.read(rxmsg))
        {
            time_CAN2 = Kernel::get_ms_count();
            wait_time = filter_msg(rxmsg);
            time_CAN += Kernel::get_ms_count() - time_CAN2;
            calls_CAN++;
        }
        if(!wait_time)
        {
            timeout++;
        }
        if(timeout > 10)
        {
            wait_time = 10;
            timeout = 0;
        }
        CANRX_thread.wait(wait_time);
    }
}

/* Filter CAN message */
uint16_t filter_msg(CANMsg& msg)
{
    uint16_t wait_time = 10;    // default wait time in ms
    if(msg.id == ACC_Msg)
    {
        msg >> data->sample_25hz.accx >> data->sample_25hz.accy >> data->sample_25hz.accz;
        wait_time = 0;
    }
    else if(msg.id == DPS_Msg)
    {
        msg >> data->sample_25hz.dpsx >> data->sample_25hz.dpsy >> data->sample_25hz.dpsz >> data->sample_25hz.dt;
       acq25hz_buffer.push(data->sample_25hz);
    }
    else if (msg.id == RPM_Msg)
    {
        msg >> data->sample_10hz.rpm >> data->sample_10hz.last_rpm;
    }
    else if(msg.id == SPD_Msg)
    {
        msg >> data->sample_10hz.speed >> data->sample_10hz.last_spd;
    }
    else if(msg.id == FLAGS_Msg)
    {
        msg >> data->chk_run;
    }

    return wait_time;
}

/* Method adapted from Kristian Lauszus library example, source: https://github.com/TKJElectronics/KalmanFilter*/
void gyro_getdata(uint16_t accx, uint16_t accy, uint16_t accz, uint16_t grx, uint16_t gry, uint16_t grz, uint16_t dt){
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

    data->roll = kalAngleX;
    data->pitch = kalAngleY;
}

/* Packet methods definition */
Packet::Packet()
{
    clear();
}

void Packet::clear()
{
    sample_25hz.accx = 0;
    sample_25hz.accy = 0;
    sample_25hz.accz = 0;
    sample_25hz.dpsx = 0;
    sample_25hz.dpsy = 0;
    sample_25hz.dpsz = 0;
    sample_25hz.dt = 0;
    sample_10hz.rpm = 0;
    sample_10hz.speed = 0;
    sample_10hz.last_spd = 0;
    sample_10hz.last_rpm = 0;
    sample_1hz.temp = 0;
    sample_1hz.count = 0;
    roll = 0;
    pitch = 0;
    chk_run = 0x00;
}