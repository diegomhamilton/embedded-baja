#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
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
/* Interrupt handlers */
void canHandler();
/* General functions*/

/* Debug variables */
Timer t;
/* OS tools */
Thread eventThread;
EventQueue queue(1024);
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
CANMsg rxMsg;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    can.attach(&canISR, CAN::RxIrq);
    choke_switch.rise(&servoSwitchISR);
    run_switch.rise(&servoSwitchISR);
    
    while (true) {
        if (switch_clicked)
        {
            switch_state = !choke_switch.read() << 1 | !run_switch.read() << 0;
            serial.printf("%d\r\n", switch_state);
            
            txMsg.clear();
            txMsg.id = 0x100;                   // set ID
            txMsg << switch_state;              // append data (8 bytes max)
            can.write(txMsg);

            switch_clicked = false;
        }
        else
        {
            wait_ms(500);
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
    // push state into state queue
}

/* Interrupt handlers */
void canHandler()
{
    led != led;                                 // debug led
    can.read(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}