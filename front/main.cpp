#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "frontdefs.h"
#include "CANMsg.h"

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
/* I/O pins */
InterruptIn choke_switch(PA_5, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PA_7, PullUp);       // servomotor RUN mode
/* Debug pins */
DigitalOut led(PC_13);

/* Interrupt services routine */
void canISR();
void servoSwitchISR();
void ticker10HzISR();
void ticker20HzISR();
void frequencyCounterISR();
/* Interrupt handlers */
void canHandler();
/* General functions*/

/* Debug variables */
Timer t;
bool buffer_full = false;
/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker10Hz;
Ticker ticker20Hz;
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
state_t current_state = IDLE_ST;
CANMsg rxMsg;

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
    ticker10Hz.attach(&ticker10HzISR, 0.1);
    ticker20Hz.attach(&ticker20HzISR, 0.05);

    while (true) {
        if (state_buffer.full())
            buffer_full = true;
        else
        {
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

        switch (current_state)
        {
            case IDLE_ST:
                Thread::wait(5);
                break;
            case SLOWACQ_ST:
                break;
            case IMU_ST:
                break;
            case SPEED_ST:
                break;
            case THROTTLE_ST:
                if (switch_clicked)
                {                    
                    switch_state = !choke_switch.read() << 1 | !run_switch.read() << 0;
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << switch_state;              // append data (8 bytes max)
                    can.write(txMsg);
                    
                    switch_clicked = false;
                }
                break;
            case DISPLAY_ST:
                break;
            case DEBUG_ST:
                serial.printf("bf=%d, cr=%d\r\n", buffer_full, switch_state);
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
    switch_clicked = true;
    state_buffer.push(THROTTLE_ST);
}

void ticker10HzISR()
{
    state_buffer.push(SPEED_ST);
}

void ticker20HzISR()
{
    state_buffer.push(IMU_ST);
}

/* Interrupt handlers */
void canHandler()
{
    led != led;                                 // debug led
    can.read(rxMsg);
    serial.printf("rx");
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}