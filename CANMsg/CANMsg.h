#ifndef CANMSG_H
#define CANMSG_H

/* CAN message container.
 * Provides "<<" (append) and ">>" (extract) operators to simplyfy
 * adding/getting data items to/from a CAN message.
 * Usage is similar to the C++ io-stream operators.
 * Data length of CAN message is automatically updated when using "<<" or ">>" operators.
 *
 * See Wiki page <https://developer.mbed.org/users/hudakz/code/CAN_Hello/> for demo.
 */
#include "CAN.h"

class CANMsg : public CANMessage
{
public:
    /** Creates empty CAN message.
     */
    CANMsg() :
        CANMessage(){ }

    /** Creates CAN message with specific content.
     */
    CANMsg(int _id, const char *_data, char _len = 8, CANType _type = CANData, CANFormat _format = CANStandard) :
        CANMessage(_id, _data, _len, _type, _format){ }

    /** Creates CAN remote message.
     */
    CANMsg(int _id, CANFormat _format = CANStandard) :
        CANMessage(_id, _format){ }

    /** Clears CAN message content
     */
    void clear(int new_id) {
        len    = 0;
        type   = CANData;
        format = CANStandard;
        id     = new_id;
        memset(data, 0, 8);
    };

    /** Append operator: Appends data (value) to CAN message
     */
    template<class T>
    CANMsg &operator<<(const T val) {
        MBED_ASSERT(len + sizeof(T) <= 8);
        //*reinterpret_cast<T*>(&data[len]) = val; // This used to work but now it hangs at run time.
        memcpy(&data[len], &val, sizeof(T));
        len += sizeof(T);
        return *this;
    }

    /** Extract operator: Extracts data (value) from CAN message
     */
    template<class T>
    CANMsg &operator>>(T& val) {
        MBED_ASSERT(sizeof(T) <= len);
        val = *reinterpret_cast<T*>(&data[0]);
        len -= sizeof(T);
        memcpy(data, data + sizeof(T), len);
        return *this;
    }
};

#endif // CANMSG_H
