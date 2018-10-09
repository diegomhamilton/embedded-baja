/* Message IDs definition */
#define SPD_Msg 0x200
#define RPM_Msg 0x201
#define ACC_Msg 0x202
#define DPS_Msg 0x203
#define TEMP_Msg 0x300
#define FLAGS_Msg 0x500

/* Masks: FLAGS message, ID: 0x500*/
#define BOX 0x80    // box call warning
#define ROV 0x40    // roll-over warning
#define STP 0x20    // vehicle stop warning
#define CHK 0x02    // choke bit
#define RUN 0x01    // run bit