#include "mbed.h"
#include "structures.h"

/* Communication protocols */
Serial uart(PA_9, PA_10);
//I2C i2c(PB_7, PB_6);
// CAN can(PB_8, PB_9);

/* Debug section */
PwmOut signal_wave(PA_8);           // simulates frequency input
//DigitalOut debug2(PB_15);
//DigitalOut debug3(PB_14);         //PB13, PB12
DigitalOut led(PC_13);

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

void main()
{
    state_t st;
    uint16_t frequency;
    uart.baud(9600);
    signal_wave.period(0.02f);                  // set frequency to 50hz
    signal_wave.write(0.5f);                    // set 50% duty cycle
    tick_25hz.attach(tick_25hz_ISR, 0.1);        // configure FREQ state ticker for 25Hz
    tick_serial.attach(tick_serial_ISR, 0.5);   // configure FREQ state ticker for 25Hz
    freq_sensor.fall(freq_ISR);                 // |
    choke.fall(choke_ISR);                      // |->attach interrupt callback
    run.fall(run_ISR);                          // |
    
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