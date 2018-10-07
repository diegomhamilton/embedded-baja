#define WHEEL_DIAMETER  584.2   // in mm
#define PI              3.14
#define STATE_BUF_SIZE  20
#define ACQ_BUF_SIZE    50

typedef enum {ACQ, CHKRUN, FREQ, SERIAL} state_t;

typedef struct {
    uint16_t accx;
    uint16_t accy;
    uint16_t accz;
    uint16_t dpsx;
    uint16_t dpsy;
    uint16_t dpsz;
    uint16_t timestamp;
} packet_100hz_t;

typedef struct {
    uint16_t speed;
    uint16_t rpm;
    uint16_t last_spd;  // time since last speed acquisition
    uint16_t last_rpm;  // time since last rpm acquisition
} packet_25hz_t;

typedef struct {
    uint8_t temp;
    uint8_t count;
} packet_1hz_t;

class Packet
{
    public:
        packet_100hz_t sample_100hz;
        packet_25hz_t sample_25hz;
        packet_1hz_t sample_1hz;
        uint8_t chk_run;                // 0x00 for disabled, 0x10 CHOKE, 0x01 RUN

    
        Packet();
        void clear();
};