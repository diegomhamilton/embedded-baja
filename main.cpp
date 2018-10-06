#include "mbed.h"
#include "structures.h"

using namespace Kernel;

/* Communication protocols */
Serial uart(PA_9, PA_10);
//I2C i2c(PB_7, PB_6);
// CAN can(PB_8, PB_9);

/* Debug section */
PwmOut signal_wave(PA_8);
//DigitalOut debug2(PB_15);
//DigitalOut debug3(PB_14); //PB13, PB12
DigitalOut led(PC_13);

/* Pins declaration */
InterruptIn choke(PA_5, PullUp);
InterruptIn run(PA_6, PullUp);
InterruptIn freq_sensor(PB_4);

/* Timers */
//Timer pulse_timer;

/* Global variables */
uint16_t pulse_counter = 0;
uint16_t current_period = 0;
uint32_t last_count = Kernel::get_ms_count();
bool choke_flag = false, run_flag = false;


/* Function declarations */
void freq_ISR();
void choke_ISR();
void run_ISR();
void chkrun_Handler();

void main()
{
    uint16_t frequency;
    uart.baud(9600);
    signal_wave.period(0.02f);  // set frequency to 50hz
    signal_wave.write(0.5f);    // set 50% duty cycle
    freq_sensor.fall(freq_ISR);
    choke.fall(choke_ISR);
    run.fall(run_ISR);
    
    while(true)
    {
        freq_sensor.fall(NULL);
        uart.printf("pulses = %d\r\n", pulse_counter);
        uart.printf("pass time = %d\r\n", current_period);
        frequency = 1000*((float)pulse_counter/current_period);
        pulse_counter = 0;
        current_period = 0;
        last_count = Kernel::get_ms_count();
        freq_sensor.fall(freq_ISR);
        uart.printf("freq = %d\r\n", frequency);
        chkrun_Handler();
        Thread::wait(500);
    }
}

void freq_ISR()
{
    pulse_counter++;                // counts pulse
    current_period += Kernel::get_ms_count() - last_count;  // sum passed time since last interrupt
    last_count = Kernel::get_ms_count();
}

void choke_ISR()
{
    choke_flag = true;
    run_flag = false;
}

void run_ISR()
{
    run_flag = true;
    choke_flag = false;
}

void chkrun_Handler()
{
    if (choke_flag && run_flag)
    {
        uart.printf("::M\r\n");
    }
    else if (!choke_flag && run_flag)
    {
        uart.printf("::R\r\n");
    }
    else if (choke_flag && !run_flag)
    {
        uart.printf("::C\r\n");
    }
    else
    {
        uart.printf("::F\r\n");
    }
}