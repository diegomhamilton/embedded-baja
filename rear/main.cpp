#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "reardefs.h"
#include "CANMsg.h"

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
/* I/O pins */
PwmOut servo(PA_6);
/* Debug pins */
DigitalOut led(PC_13);

/* Interrupt services routine */
void canISR();
/* Interrupt handlers */
void canHandler();
/* General functions*/

/* Debug variables */
Timer t;
/* OS tools */
Thread eventThread(osPriorityBelowNormal, 1000);
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
    servo.period_ms(20);                        // set signal frequency to 50Hz
    servo.write(0);                             // disables servo
    
    while (true) {
        if (switch_clicked)
        {
            serial.printf("%d", switch_state);
            switch (switch_state)
            {
                case 0x00:
                    servo.pulsewidth_us(SERVO_MID);
                    break;
                case 0x01:
                    servo.pulsewidth_us(SERVO_RUN);
                    break;
                case 0x02:
                    servo.pulsewidth_us(SERVO_CHOKE);
                    break;
                default:
                    serial.printf("Choke/run error\r\n");
                    break;
            }

            txMsg.clear();
            txMsg.id = 0x101;                   // set ID
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

/* Interrupt handlers */
void canHandler()
{
    led != led;                                 // debug led
    can.read(rxMsg);
    switch_clicked = true;
    rxMsg >> switch_state;
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}