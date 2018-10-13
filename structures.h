#define WHEEL_DIAMETER  584.2   // in mm
#define PI              3.1416
#define RAD_TO_DEGREE   180.0/PI
#define TO_G          2.0/32768.0
#define TO_DPS          245.0/32768.0
#define STATE_BUF_SIZE  40
#define ACQ_BUF_SIZE    50

typedef enum {ACQ, CHKRUN, FREQ, SERIAL, IDLE} state_t;

typedef struct {
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t dpsx;
    int16_t dpsy;
    int16_t dpsz;
    uint16_t dt;        // time since last acquisition
} packet_25hz_t;

typedef struct {
    uint16_t speed;
    uint16_t rpm;
    uint16_t last_spd;  // time since last speed acquisition
    uint16_t last_rpm;  // time since last rpm acquisition
} packet_10hz_t;

typedef struct {
    uint8_t temp;
    uint8_t count;
} packet_1hz_t;

class Packet
{
    public:
        packet_25hz_t sample_25hz;
        packet_10hz_t sample_10hz;
        packet_1hz_t sample_1hz;
        int16_t roll;
        int16_t pitch;
        uint8_t chk_run;                // 0x00 for disabled, 0x02 CHOKE, 0x01 RUN

    
        Packet();
        void clear();
};