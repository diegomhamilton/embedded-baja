#include "mbed.h"
#include "structures.h"
#include "CANMsg.h"
#include "CANDef.h"

/* Communication protocols */
CAN can(PB_8, PB_9);
Serial uart(PA_9, PA_10);
//I2C i2c(PB_7, PB_6);

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
InterruptIn choke(PA_5, PullUp);
InterruptIn run(PA_6, PullUp);
InterruptIn freq_sensor(PB_4);
// AnalogIn    temperature();

/* Timers */
Ticker tick_25hz;
Ticker tick_serial;
//Timer pulse_timer;

/* Global variables */
Packet* data = new Packet();
uint16_t pulse_counter = 0;
uint16_t current_period = 0;
uint32_t last_count = Kernel::get_ms_count();                   // store moment of last interrupt (PB_4)
bool choke_flag = false, run_flag = false;
CircularBuffer <state_t, STATE_BUF_SIZE> state_buffer;          // store sequence of events to proccess
CircularBuffer <packet_100hz_t, ACQ_BUF_SIZE> acq100hz_buffer;  // store packets of accelerometer and gyro

/* Function declarations */
void freq_ISR();
void choke_ISR();
void run_ISR();
void tick_25hz_ISR();
void tick_serial_ISR();
void CANRX_check();
uint16_t filter_msg(CANMsg& msg);

void main()
{
    /* Variables declaration */
    state_t st;
    uint16_t frequency;
    /* RTOS set up */
    osThreadId id = osThreadGetId();                        //get id from main thread
    osThreadSetPriority(osThreadGetId(), osPriorityNormal); // set priority of main thread 1 level before CANRX_thread
    /* Initialization of peripherals (communication, tickers, interrupts) */
    uart.baud(9600);                            // set UART baud to 9600 (change do 115200)
    can.frequency(5000000);                    // set CAN rate to 1Mbps
    signal_wave.period(0.02f);                  // set frequency to 50hz
    signal_wave.write(0.5f);                    // set 50% duty cycle
    tick_25hz.attach(tick_25hz_ISR, 0.1);       // configure FREQ state ticker for 10Hz
    tick_serial.attach(tick_serial_ISR, 0.5);   // configure SERIAL state ticker for 2Hz
    freq_sensor.fall(freq_ISR);                 // |
    choke.fall(choke_ISR);                      // |->attach interrupt callback
    run.fall(run_ISR);                          // |
    
    CANRX_thread.start(CANRX_check);

    while(true)
    {
        if(!state_buffer.empty())
            state_buffer.pop(st);
        else
            st = ACQ;
        
        switch(st)
        {
            case ACQ:
                uart.printf("*\r\n");
                Thread::wait(20);
                // get data from accelerometer
                // send data / calculate roll and pitch
                break;
            case CHKRUN:
                if (choke.read() && !run.read())
                {
                    uart.printf("::R\r\n");
                }
                else if (!choke.read() && run.read())
                {
                    uart.printf("::C\r\n");
                }
                else if (choke.read() && run.read())
                {
                    uart.printf("::M\r\n");
                }
                else
                {
                    uart.printf("::F\r\n");
                }
                break;
            case FREQ:
                freq_sensor.fall(NULL);
                data->sample_25hz.last_spd = 1000*((float)pulse_counter/current_period);    //calculates frequency in Hz
                // makes conversion to speed
                pulse_counter = 0;
                current_period = 0;
                last_count = Kernel::get_ms_count();
                freq_sensor.fall(freq_ISR);
                break;
            case SERIAL:
                // formats string to send to display or radio
                uart.printf("speed: %d\r\n", data->sample_25hz.last_spd);
                uart.printf("dpsx = %d\r\n", data->sample_100hz.dpsx);
                uart.printf("CAN avg time = %f\r\n", (float)time_CAN/calls_CAN);
                calls_CAN = 0;
                break;
            default:
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

void tick_25hz_ISR()
{
    state_buffer.push(FREQ);
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
        // todo implement timeout for wait_time == 0
        CANRX_thread.wait(wait_time);
    }
}

/* Filter CAN message */
uint16_t filter_msg(CANMsg& msg)
{
    uint16_t wait_time = 10;    // default wait time in ms
    if(msg.id == ACC_Msg)
    {
        msg >> data->sample_100hz.accx >> data->sample_100hz.accy >> data->sample_100hz.accz;
        wait_time = 0;
    }
    else if(msg.id == DPS_Msg)
    {
        msg >> data->sample_100hz.dpsx >> data->sample_100hz.dpsy >> data->sample_100hz.dpsz >> data->sample_100hz.timestamp;
        acq100hz_buffer.push(data->sample_100hz);
    }
    else if (msg.id == RPM_Msg)
    {
        msg >> data->sample_25hz.rpm >> data->sample_25hz.last_rpm;
    }
    else if(msg.id == SPD_Msg)
    {
        msg >> data->sample_25hz.speed >> data->sample_25hz.last_spd;
    }

    return wait_time;
}

/* Packet methods definition */
Packet::Packet()
{
    clear();
}

void Packet::clear()
{
    sample_100hz.accx = 0;
    sample_100hz.accy = 0;
    sample_100hz.accz = 0;
    sample_100hz.dpsx = 0;
    sample_100hz.dpsy = 0;
    sample_100hz.dpsz = 0;
    sample_100hz.timestamp = 0;
    sample_25hz.rpm = 0;
    sample_25hz.speed = 0;
    sample_25hz.last_spd = 0;
    sample_25hz.last_rpm = 0;
    sample_1hz.temp = 0;
    sample_1hz.count = 0;
    chk_run = 0x00;
}