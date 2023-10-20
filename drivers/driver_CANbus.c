#include "device.h"
#include "peripheral/canfd/plib_canfd1.h"
#include "VCU_CANdata.h"
#include "diagnostics.h"
#include <sys/kmem.h>

#define CANSTB_PIN LATCbits.LATC15

void CANbus_init(void)
{
    CANSTB_PIN = 0;         //Enable CABbus transreciever
}

void CANbus_write(uint8_t Sadr, uint8_t * Sdata, uint8_t Sdata_L )
{
    bool sstat = 0;
    sstat = CAN1_MessageTransmit(Sadr, Sdata_L, Sdata, 1, 0, 0);
    sstat = sstat;
}

void CANbus_read(void)
{
    uint8_t Rdata[8];
    bool rstat = 0;
    uint8_t msg_len = 0;
    uint32_t msg_id = 0;
    uint32_t timestamp = 0;
    CANFD_MSG_RX_ATTRIBUTE atri = 0;
    uint8_t APPstatus = 0x02;    
    uint32_t devsn0, devsn1;
    uint8_t b1data[8] = {0,0,0,0, 0,0,0,0};
    
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
            
            case 0xB0:
                Nop();
                Nop();
                CANbus_write(0xB0, &APPstatus, 1);
            break;
            
            case 0xB1:
                devsn0 = DEVSN0;
                devsn1 = DEVSN1;
                __builtin_memcpy((uint32_t *)(b1data+0), (uint32_t *)&devsn0, 4);
                __builtin_memcpy((uint32_t *)(b1data+4), (uint32_t *)&devsn1, 4);
                CANbus_write(0xB1, b1data, 8);
            break;
            
            
            case 0xC0:
                C0_send_TU_data_CAN();
            break;
            
            default:
                Nop();
            break;
        }
    }
    Nop();
}



// *****************************************************************************
// *****************************************************************************
// Global Data
// *****************************************************************************
// *****************************************************************************
/* CAN1 Message memory size */
#define CANFD_MESSAGE_RAM_CONFIG_SIZE 512
/* Number of configured FIFO */
#define CANFD_NUM_OF_FIFO             2
/* Maximum number of CAN Message buffers in each FIFO */
#define CANFD_FIFO_MESSAGE_BUFFER_MAX 32

#define CANFD_CONFIGURATION_MODE      0x4
#define CANFD_OPERATION_MODE          0x6
#define CANFD_NUM_OF_FILTER           1
/* FIFO Offset in word (4 bytes) */
#define CANFD_FIFO_OFFSET             0xc
/* Filter Offset in word (4 bytes) */
#define CANFD_FILTER_OFFSET           0x4
#define CANFD_FILTER_OBJ_OFFSET       0x8
/* Acceptance Mask Offset in word (4 bytes) */
#define CANFD_ACCEPTANCE_MASK_OFFSET  0x8
#define CANFD_MSG_SID_MASK            0x7FF
#define CANFD_MSG_EID_MASK            0x1FFFFFFF
#define CANFD_MSG_DLC_MASK            0x0000000F
#define CANFD_MSG_IDE_MASK            0x00000010
#define CANFD_MSG_RTR_MASK            0x00000020
#define CANFD_MSG_BRS_MASK            0x00000040
#define CANFD_MSG_FDF_MASK            0x00000080
#define CANFD_MSG_SEQ_MASK            0xFFFFFE00
#define CANFD_MSG_TX_EXT_SID_MASK     0x1FFC0000
#define CANFD_MSG_TX_EXT_EID_MASK     0x0003FFFF
#define CANFD_MSG_RX_EXT_SID_MASK     0x000007FF
#define CANFD_MSG_RX_EXT_EID_MASK     0x1FFFF800
#define CANFD_MSG_FLT_EXT_SID_MASK    0x1FFC0000
#define CANFD_MSG_FLT_EXT_EID_MASK    0x0003FFFF

static uint8_t __attribute__((coherent, aligned(16))) can_message_buffer[CANFD_MESSAGE_RAM_CONFIG_SIZE];

void CAN1_Initialize500(void)
{
    /* Switch the CAN module ON */
    CFD1CON |= _CFD1CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD1CON = (CFD1CON & ~_CFD1CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD1CON_REQOP_POSITION) & _CFD1CON_REQOP_MASK);
    while(((CFD1CON & _CFD1CON_OPMOD_MASK) >> _CFD1CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE);

    /* Set the Nominal bitrate to 500 Kbps */
    CFD1NBTCFG = ((0 << _CFD1NBTCFG_BRP_POSITION) & _CFD1NBTCFG_BRP_MASK)
               | ((118 << _CFD1NBTCFG_TSEG1_POSITION) & _CFD1NBTCFG_TSEG1_MASK)
               | ((119 << _CFD1NBTCFG_TSEG2_POSITION) & _CFD1NBTCFG_TSEG2_MASK)
               | ((11 << _CFD1NBTCFG_SJW_POSITION) & _CFD1NBTCFG_SJW_MASK);

    /* Set Message memory base address for all FIFOs/Queue */
    CFD1FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    CFD1CON &= ~_CFD1CON_STEF_MASK;

    CFD1CON &= ~_CFD1CON_TXQEN_MASK;


    /* Configure CAN FIFOs */
    CFD1FIFOCON1 = (((16 - 1) << _CFD1FIFOCON1_FSIZE_POSITION) & _CFD1FIFOCON1_FSIZE_MASK) | _CFD1FIFOCON1_TXEN_MASK | ((0x1c << _CFD1FIFOCON1_TXPRI_POSITION) & _CFD1FIFOCON1_TXPRI_MASK) | ((0x0 << _CFD1FIFOCON1_RTREN_POSITION) & _CFD1FIFOCON1_RTREN_MASK) | ((0x0 << _CFD1FIFOCON1_PLSIZE_POSITION) & _CFD1FIFOCON1_PLSIZE_MASK);
    CFD1FIFOCON2 = (((16 - 1) << _CFD1FIFOCON2_FSIZE_POSITION) & _CFD1FIFOCON2_FSIZE_MASK) | ((0x0 << _CFD1FIFOCON2_PLSIZE_POSITION) & _CFD1FIFOCON2_PLSIZE_MASK);

    /* Configure CAN Filters */
    /* Filter 0 configuration */
    CFD1FLTOBJ0 = (0 & CANFD_MSG_SID_MASK);
    CFD1MASK0 = (0 & CANFD_MSG_SID_MASK);
    CFD1FLTCON0 |= (((0x2 << _CFD1FLTCON0_F0BP_POSITION) & _CFD1FLTCON0_F0BP_MASK)| _CFD1FLTCON0_FLTEN0_MASK);

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD1CON = (CFD1CON & ~_CFD1CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD1CON_REQOP_POSITION) & _CFD1CON_REQOP_MASK);
    while(((CFD1CON & _CFD1CON_OPMOD_MASK) >> _CFD1CON_OPMOD_POSITION) != CANFD_OPERATION_MODE);
}

void CAN1_Initialize250(void)
{
    /* Switch the CAN module ON */
    CFD1CON |= _CFD1CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD1CON = (CFD1CON & ~_CFD1CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD1CON_REQOP_POSITION) & _CFD1CON_REQOP_MASK);
    while(((CFD1CON & _CFD1CON_OPMOD_MASK) >> _CFD1CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE);

    /* Set the Nominal bitrate to 250 Kbps */
    CFD1NBTCFG = ((1 << _CFD1NBTCFG_BRP_POSITION) & _CFD1NBTCFG_BRP_MASK)
               | ((118 << _CFD1NBTCFG_TSEG1_POSITION) & _CFD1NBTCFG_TSEG1_MASK)
               | ((119 << _CFD1NBTCFG_TSEG2_POSITION) & _CFD1NBTCFG_TSEG2_MASK)
               | ((11 << _CFD1NBTCFG_SJW_POSITION) & _CFD1NBTCFG_SJW_MASK);

    /* Set Message memory base address for all FIFOs/Queue */
    CFD1FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    CFD1CON &= ~_CFD1CON_STEF_MASK;

    CFD1CON &= ~_CFD1CON_TXQEN_MASK;


    /* Configure CAN FIFOs */
    CFD1FIFOCON1 = (((16 - 1) << _CFD1FIFOCON1_FSIZE_POSITION) & _CFD1FIFOCON1_FSIZE_MASK) | _CFD1FIFOCON1_TXEN_MASK | ((0x1c << _CFD1FIFOCON1_TXPRI_POSITION) & _CFD1FIFOCON1_TXPRI_MASK) | ((0x0 << _CFD1FIFOCON1_RTREN_POSITION) & _CFD1FIFOCON1_RTREN_MASK) | ((0x0 << _CFD1FIFOCON1_PLSIZE_POSITION) & _CFD1FIFOCON1_PLSIZE_MASK);
    CFD1FIFOCON2 = (((16 - 1) << _CFD1FIFOCON2_FSIZE_POSITION) & _CFD1FIFOCON2_FSIZE_MASK) | ((0x0 << _CFD1FIFOCON2_PLSIZE_POSITION) & _CFD1FIFOCON2_PLSIZE_MASK);

    /* Configure CAN Filters */
    /* Filter 0 configuration */
    CFD1FLTOBJ0 = (0 & CANFD_MSG_SID_MASK);
    CFD1MASK0 = (0 & CANFD_MSG_SID_MASK);
    CFD1FLTCON0 |= (((0x2 << _CFD1FLTCON0_F0BP_POSITION) & _CFD1FLTCON0_F0BP_MASK)| _CFD1FLTCON0_FLTEN0_MASK);

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD1CON = (CFD1CON & ~_CFD1CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD1CON_REQOP_POSITION) & _CFD1CON_REQOP_MASK);
    while(((CFD1CON & _CFD1CON_OPMOD_MASK) >> _CFD1CON_OPMOD_POSITION) != CANFD_OPERATION_MODE);
}