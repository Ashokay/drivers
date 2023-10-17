#include "device.h"
#include"config/V3_MCF1024/peripheral/canfd/plib_canfd1.h"
#include "VCU_CANdata.h"

#define CANSTB_PIN LATCbits.LATC15

void CANbus_init(void)
{
    CANSTB_PIN = 0;         //Enable CABbus transreciever
}

void CANbus_write(uint8_t Sadr, uint8_t * Sdata, uint8_t Sdata_L )
{
    //uint8_t cansdata[4] = {45, 46,47,48};
    bool sstat = 0;
    sstat = CAN1_MessageTransmit(Sadr, Sdata_L, Sdata, 1, 0, 0);
    sstat = sstat;
}

void CANbus_read(void)
{
    uint8_t Rdata[8];
    bool rstat = 0;
    uint8_t msg_len = 0;
    static long int time_count = 0;
    uint32_t msg_id = 0;
    uint32_t timestamp = 0;
    CANFD_MSG_RX_ATTRIBUTE atri = 0;
    
    Nop();
    Nop();
    
    
    if(time_count == 0) //test
    {
        //
        
        //A1_action(Rdata, msg_len);
        //send_A2data();
        time_count--;
    }
        
    rstat = CAN1_MessageReceive(&msg_id, &msg_len, &Rdata[0], &timestamp, 2, &atri);
    if(rstat)
    {
        rstat = 0;
        Nop();
        Nop();
        Nop();
        switch(msg_id)
        {
            case 0xA1:
                A1_action(Rdata, msg_len);
            break;
            
            case 0xA3:
                A3_action(Rdata, msg_len);
            break;
            
            default:
                Nop();
            break;
        }
    }
    Nop();
}