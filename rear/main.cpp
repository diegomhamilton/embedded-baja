#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "reardefs.h"
#include "CANMsg.h"
#include "RFM69.h"

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200); 
RFM69 radio(PB_15, PB_14, PB_13, PB_12, PA_8); // RFM69::RFM69(PinName  PinName mosi, PinName miso, PinName sclk,slaveSelectPin, PinName int)
/* I/O pins */
AnalogIn analog(PA_0);
InterruptIn freq_sensor(PB_5);
PwmOut servo(PA_6);
/* Debug pins */
PwmOut signal(PA_7);
DigitalOut led(PC_13);
DigitalOut dbg1(PC_14);
DigitalOut dbg2(PC_15);
DigitalOut dbg3(PA_1);
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
CircularBuffer <state_t, 2*BUFFER_SIZE> state_buffer;
CircularBuffer <imu_t*, 20> imu_buffer;
CircularBuffer <acq_10hz_t*, 10> d10hz_buffer;
CircularBuffer <temperature_t, 10> temp_buffer;
CircularBuffer <packet_t, 10> radio_buffer;

/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
state_t current_state = IDLE_ST;
uint8_t pulse_counter = 0;
uint64_t current_period = 0, last_count = 0;
float V_termistor = 0;
packet_t data;                                // Create package for radio comunication
packet_t radio_packet;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    servo.period_ms(20);                        // set signal frequency to 50Hz
    servo.write(0);                             // disables servo
    signal.period_ms(32);                       // set signal frequency to 1/0.032Hz
    signal.write(0.5f);                         // dutycycle 50%
    radio.initialize(FREQUENCY_915MHZ, NODE_ID, NETWORK_ID);
    radio.encrypt(0);
    radio.setPowerLevel(20);
    can.attach(&canISR, CAN::RxIrq);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    ticker10Hz.attach(&ticker10HzISR, 0.1);
    freq_sensor.fall(&frequencyCounterISR);
    
    while (true) {
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
//              Thread::wait(1);
                break;
            case SLOWACQ_ST:
                dbg1 = !dbg1;
                V_termistor = VCC*analog.read();
                data.temp.motor = (uint16_t)((float) (1.0/0.032)*log((1842.8*(VCC - V_termistor)/(V_termistor*R_TERM))));
                temp_buffer.push(data.temp);
                state_buffer.push(DEBUG_ST);
                break;
            case RPM_ST:
                dbg2 = !dbg2;
                freq_sensor.fall(NULL);         // disable interrupt
                if (current_period != 0)
                {
                    data.data_10hz[packet_counter[N_RPM]].rpm = 1000000*((float)pulse_counter/current_period);    //calculates frequency in Hz
                }
                else
                {
                    data.data_10hz[packet_counter[N_RPM]].rpm = 0;
                }
                pulse_counter = 0;                          
                current_period = 0;                                   // reset pulses related variables
                last_count = t.read_us();        
                freq_sensor.fall(&frequencyCounterISR);               // enable interrupt
                
                if (packet_counter[N_RPM] < 1)
                {
                    packet_counter[N_RPM]++;
                }
                else if (packet_counter[N_RPM] == 1)
                {   
                    d10hz_buffer.push(data.data_10hz);
                    packet_counter[N_RPM] = 0;
                }
                break;
            case THROTTLE_ST:
                if (switch_clicked)
                {
                    data.data_10hz[packet_counter[N_FLAG]].flags &= ~(0x07);         // reset servo-related flags
                    switch (switch_state)
                    {
                        case 0x00:
                            dbg3 = !dbg3;
                            servo.pulsewidth_us(SERVO_MID);
                            data.data_10hz[packet_counter[N_FLAG]].flags &= ~(0x03); // reset run and choke flags
                            break;
                        case 0x01:
                            dbg3 = !dbg3;
                            servo.pulsewidth_us(SERVO_RUN);
                            data.data_10hz[packet_counter[N_FLAG]].flags |= 0x01;    // set run flag
                            break;
                        case 0x02:
                            dbg3 = !dbg3;
                            servo.pulsewidth_us(SERVO_CHOKE);
                            data.data_10hz[packet_counter[N_FLAG]].flags |= 0x02;    // set choke flag
                            break;
                        default:
                            serial.printf("Choke/run error\r\n");
                            data.data_10hz[packet_counter[N_FLAG]].flags |= 0x04;    // set servo error flag
                            break;
                    }

                    switch_clicked = false;
                }
                
                if (packet_counter[N_FLAG] <1)
                {
                    packet_counter[N_FLAG]++;
                }
                else if (packet_counter[N_FLAG] == 1)
                {
                    packet_counter[N_FLAG] = 0;
                }
                break;
            case RADIO_ST:
                if((!imu_buffer.empty()) && (!d10hz_buffer.empty()) && (!temp_buffer.empty()))
                {
                    
                    imu_t* temp_imu;
                    imu_buffer.pop(temp_imu);
                    acq_10hz_t* temp_10hz;
                    d10hz_buffer.pop(temp_10hz);
                    memcpy(&radio_packet.imu,temp_imu, 4*sizeof(imu_t));
                    memcpy(&radio_packet.data_10hz,temp_10hz, 2*sizeof(acq_10hz_t));
                    temp_buffer.pop(radio_packet.temp);
                }
            
                radio.send((uint8_t)BOXRADIO_ID, &radio_packet, sizeof(packet_t), true);     // request ACK with 1 retry (waitTime = 40ms)
                break;
            case DEBUG_ST:
//                serial.printf("radio state pushed");
                  state_buffer.push(RADIO_ST);
//                serial.printf("bf=%d, cr=%d\r\n", buffer_full, switch_state);
//                serial.printf("speed=%d\r\n", data.data_10hz[packet_counter[N_SPEED]].speed);
//                serial.printf("rpm=%d\r\n", data.data_10hz[packet_counter[N_RPM]].rpm);
//                serial.printf("imu acc x =%d\r\n", data.imu[packet_counter[N_IMU]].acc_x);
//                serial.printf("imu acc y =%d\r\n", data.imu[packet_counter[N_IMU]].acc_y);
//                serial.printf("imu acc z =%d\r\n", data.imu[packet_counter[N_IMU]].acc_z);
//                serial.printf("imu dps x =%d\r\n", data.imu[packet_counter[N_IMU]].dps_x);
//                serial.printf("imu dps y =%d\r\n", data.imu[packet_counter[N_IMU]].dps_y);
//                serial.printf("imu dps z =%d\r\n", data.imu[packet_counter[N_IMU]].dps_z);
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
    if (msg.id == THROTTLE_ID)
    {
        switch_clicked = true;
        state_buffer.push(THROTTLE_ST);
        msg >> switch_state;
    }
    else if (msg.id == IMU_ACC_ID)
    {
        msg >> data.imu[packet_counter[N_IMU]].acc_x >> data.imu[packet_counter[N_IMU]].acc_y
                         >> data.imu[packet_counter[N_IMU]].acc_z;
    }
    else if (msg.id == IMU_DPS_ID)
    {
        msg >> data.imu[packet_counter[N_IMU]].dps_x >> data.imu[packet_counter[N_IMU]].dps_y
                         >> data.imu[packet_counter[N_IMU]].dps_z;
        if (packet_counter[N_IMU] <3)
        {
            packet_counter[N_IMU]++;
        }
        else if (packet_counter[N_IMU] == 3)
        {
            imu_buffer.push(data.imu);
            packet_counter[N_IMU] = 0;
        }
    }
    else if (msg.id == SPEED_ID)
    {
        msg >> data.data_10hz[packet_counter[N_SPEED]].speed;
    }
}
