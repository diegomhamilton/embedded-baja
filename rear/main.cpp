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
void servoSwitchISR();
void ticker5HzISR();
void ticker10HzISR();
void frequencyCounterISR();
/* Interrupt handlers */
void canHandler();
/* General functions*/
void filterMessage(CANMsg msg);

/* Debug variables */
Timer t;
bool buffer_full = false;
/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker5Hz;
Ticker ticker10Hz;
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
state_t current_state = IDLE_ST;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    servo.period_ms(20);                        // set signal frequency to 50Hz
    servo.write(0);                             // disables servo
    can.attach(&canISR, CAN::RxIrq);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    ticker10Hz.attach(&ticker10HzISR, 0.1);

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
            case RPM_ST:
                break;
            case THROTTLE_ST:
                if (switch_clicked)
                {
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

                    switch_clicked = false;
                }
                break;
            case RADIO_ST:
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

void ticker5HzISR()
{
    state_buffer.push(SLOWACQ_ST);
}

void ticker10HzISR()
{
    state_buffer.push(RPM_ST);
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