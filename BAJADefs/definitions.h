#define CAN_RF0R        (*((volatile unsigned long *)0x4000640C))
#define CAN_FLAG_FMP0   ((uint32_t)0x12000003) /*!< FIFO 0 Message Pending Flag */
#define CAN_RF0R_RFOM0 ((uint8_t)0x20) /*!<Release FIFO 0 Output Mailbox */
#define CAN_RF1R        (*((volatile unsigned long *)0x40006410))
#define CAN_IER         (*((volatile unsigned long *)0x40006414))
#define CAN_IER_FMPIE0  ((uint32_t)0x00000002) /*!<FIFO Message Pending Interrupt Enable */