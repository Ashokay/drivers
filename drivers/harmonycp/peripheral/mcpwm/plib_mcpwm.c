/*******************************************************************************
  Motor Control PWM (MCPWM) Peripheral Library (PLIB)

  Company:
    Microchip Technology Inc.

  File Name:
    plib_mcpwm.c

  Summary:
    MCPWM Source File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
#include "device.h"
#include "plib_mcpwm.h"
#include "state_engine.h"

// *****************************************************************************

// *****************************************************************************
// Section: MCPWM Implementation
// *****************************************************************************
// *****************************************************************************
MCPWM_OBJECT mcpwmPriEventObj;

void MCPWM_Initialize (void)
{
    /* PTCON register  */
    /*  SEVTPS  = 1 */
    /*  PCLKDIV = 0 */
    /*  SEIEN   = true */
    PTCON = 0x801;
    
    /* STCON register  */
    /*  SEVTPS  = 0 */
    /*  SCLKDIV = 0 */
    /*  SSEIEN = false */
    STCON = 0x0;
    
    PTPER = 2400;
    STPER = 2000;
    SEVTCMP = 1200;
    SSEVTCMP = 10;

    /*********** Channel 1 Configurations **********/
    /* PWMCON1 register  */
    /*  MTBS   = 0 */
    /*  PTDIR  =  0 */
    /*  ECAM   =  1 */
    /*  DTCP   =  0 */
    /*  DTC    =  0 */
    /*  ITB    = 0 */
    /*  PWMHIEN =  false */
    /*  PWMLIEN = false */
    /*  TRGIEN = false */
    /*  CLIEN = false */
    /*  FLTIEN = false */
    PWMCON1 = 0x400;

    /* IOCON1 register  */
    /*  SWAP    = 0*/
    /*  PMOD    = 0*/
    /*  POLH    = 0*/
    /*  POLL    = 0*/
    /*  FLTDAT  = 0b00 */
    /*  FLTMOD  = 3 */
    /*  PENH  = 1 */
    /*  PENL  = 1 */
    /*  FLTPOL  = 0  */
    /*  FLTSRC  = 0  */
    /*  CLDAT  = 0b00 */
    /*  CLMOD  = 0 */
    /*  CLPOL  = 0  */
    /*  CLSRC  = 0  */
    IOCON1 = 0xc000;

    PDC1 = 1000;
    SDC1 = 500;
    PHASE1 = 0;

    /* Dead Time */
    DTR1 = 240;
    ALTDTR1 = 240;

    /* Trigger Generator */
    TRGCON1 = 0x0;
    TRIG1 = 0;
    STRIG1 = 0;
    
    /* leading edge blanking */
    /* LEBCON1 register  */
    /*  CLLEBEN    = false  */
    /*  FLTLEBEN   = false */
    /*  PLF        = 0  */
    /*  PLR        = 0  */
    /*  PHF        = 0  */
    /*  PHR        = 0  */
    LEBCON1 = 0x0;
    LEBDLY1 = 10;


    /*********** Channel 2 Configurations **********/
    /* PWMCON2 register  */
    /*  MTBS   = 0 */
    /*  PTDIR  =  0 */
    /*  ECAM   =  1 */
    /*  DTCP   =  0 */
    /*  DTC    =  0 */
    /*  ITB    = 0 */
    /*  PWMHIEN =  false */
    /*  PWMLIEN = false */
    /*  TRGIEN = false */
    /*  CLIEN = false */
    /*  FLTIEN = false */
    PWMCON2 = 0x400;

    /* IOCON2 register  */
    /*  SWAP    = 0*/
    /*  PMOD    = 0*/
    /*  POLH    = 0*/
    /*  POLL    = 0*/
    /*  FLTDAT  = 0b00 */
    /*  FLTMOD  = 3 */
    /*  PENH  = 1 */
    /*  PENL  = 1 */
    /*  FLTPOL  = 0  */
    /*  FLTSRC  = 0  */
    /*  CLDAT  = 0b00 */
    /*  CLMOD  = 0 */
    /*  CLPOL  = 0  */
    /*  CLSRC  = 0  */
    IOCON2 = 0xc000;

    PDC2 = 1000;
    SDC2 = 500;
    PHASE2 = 0;

    /* Dead Time */
    DTR2 = 240;
    ALTDTR2 = 240;

    /* Trigger Generator */
    TRGCON2 = 0x0;
    TRIG2 = 0;
    STRIG2 = 0;

    /* leading edge blanking */
    /* LEBCON2 register  */
    /*  CLLEBEN    = false  */
    /*  FLTLEBEN   = false */
    /*  PLF        = 0  */
    /*  PLR        = 0  */
    /*  PHF        = 0  */
    /*  PHR        = 0  */
    LEBCON2 = 0x0;
    LEBDLY2 = 10;


    /*********** Channel 3 Configurations **********/
    /* PWMCON3 register  */
    /*  MTBS   = 0 */
    /*  PTDIR  =  0 */
    /*  ECAM   =  1 */
    /*  DTCP   =  0 */
    /*  DTC    =  0 */
    /*  ITB    = 0 */
    /*  PWMHIEN =  false */
    /*  PWMLIEN = false */
    /*  TRGIEN = false */
    /*  CLIEN = false */
    /*  FLTIEN = false */
    PWMCON3 = 0x400;

    /* IOCON3 register  */
    /*  SWAP    = 0*/
    /*  PMOD    = 0*/
    /*  POLH    = 0*/
    /*  POLL    = 0*/
    /*  FLTDAT  = 0b00 */
    /*  FLTMOD  = 3 */
    /*  PENH  = 1 */
    /*  PENL  = 1 */
    /*  FLTPOL  = 0  */
    /*  FLTSRC  = 0  */
    /*  CLDAT  = 0b00 */
    /*  CLMOD  = 0 */
    /*  CLPOL  = 0  */
    /*  CLSRC  = 0  */
    IOCON3 = 0xc000;

    PDC3 = 1000;
    SDC3 = 500;
    PHASE3 = 0;

    /* Dead Time */
    DTR3 = 240;
    ALTDTR3 = 240;

    /* Trigger Generator */
    TRGCON3 = 0x0;
    TRIG3 = 0;
    STRIG3 = 0;

    /* leading edge blanking */
    /* LEBCON3 register  */
    /*  CLLEBEN    = false  */
    /*  FLTLEBEN   = false */
    /*  PLF        = 0  */
    /*  PLR        = 0  */
    /*  PHF        = 0  */
    /*  PHR        = 0  */
    LEBCON3 = 0x0;
    LEBDLY3 = 10;


    /* Enable primary special event interrupt */
    IEC5SET = _IEC5_PWMPEVTIE_MASK;
    mcpwmPriEventObj.callback = NULL;
}


void MCPWM_Start(void)
{
    /* Enable MCPWM module */
    PTCONbits.PTEN = 0x1;
}

void MCPWM_Stop(void)
{
    /* Disable MCPWM module */
    PTCONbits.PTEN = 0x0;
}

void MCPWM_PrimaryPeriodSet(uint16_t period)
{
    PTPER = period;
}

uint16_t MCPWM_PrimaryPeriodGet(void)
{
    return PTPER;
}


void MCPWM_SecondaryPeriodSet(uint16_t period)
{
    STPER = period;
}

uint16_t MCPWM_SecondaryPeriodGet(void)
{
    return STPER;
}

void MCPWM_ChannelPrimaryDutySet(MCPWM_CH_NUM channel, uint16_t duty)
{
    *(&PDC1 + (0x40 * (channel))) = duty;
}

void MCPWM_ChannelSecondaryDutySet(MCPWM_CH_NUM channel, uint16_t duty)
{
    *(&SDC1 + (0x40 * (channel))) = duty;
}

void MCPWM_ChannelDeadTimeSet(MCPWM_CH_NUM channel, uint16_t high_deadtime, uint16_t low_deadtime)
{
    *(&DTR1 + (0x40 * (channel))) = (high_deadtime & 0x3FFF);
    *(&ALTDTR1 + (0x40 * (channel))) = (low_deadtime & 0x3FFF);
}

void MCPWM_ChannelPrimaryTriggerSet(MCPWM_CH_NUM channel, uint16_t trigger)
{
    *(&TRIG1 + (0x40 * (channel))) = trigger;
}

void MCPWM_ChannelSecondaryTriggerSet(MCPWM_CH_NUM channel, uint16_t trigger)
{
    *(&STRIG1 + (0x40 * (channel))) = trigger;
}

void MCPWM_ChannelLeadingEdgeBlankingDelaySet(MCPWM_CH_NUM channel, uint16_t delay)
{
    *(&LEBDLY1 + (0x40 * (channel))) = delay;
}

void MCPWM_ChannelPinsOverrideEnable(MCPWM_CH_NUM channel)
{
    *(&IOCON1 + (0x40 * (channel))) |= _IOCON1_OVRENL_MASK | _IOCON1_OVRENH_MASK;
}

void MCPWM_ChannelPinsOverrideDisable(MCPWM_CH_NUM channel)
{
    *(&IOCON1 + (0x40 * (channel))) &= ~(_IOCON1_OVRENL_MASK | _IOCON1_OVRENH_MASK);
}

void MCPWM_ChannelPinsOwnershipEnable(MCPWM_CH_NUM channel)
{
    *(&IOCON1 + (0x40 * (channel))) |= _IOCON1_PENH_MASK | _IOCON1_PENL_MASK;
}

void MCPWM_ChannelPinsOwnershipDisable(MCPWM_CH_NUM channel)
{
    *(&IOCON1 + (0x40 * (channel))) &= ~(_IOCON1_PENH_MASK | _IOCON1_PENL_MASK);
}

void PWM_PRI_InterruptHandler(void)
{
    PTCONbits.SEIEN = 0;
    IFS5CLR = _IFS5_PWMPEVTIF_MASK;    //Clear IRQ flag
    PTCONbits.SEIEN = 1;
    
    fast_routines();

    if( (mcpwmPriEventObj.callback != NULL))
    {
        mcpwmPriEventObj.callback(mcpwmPriEventObj.context);
    }
}

void MCPWM_PrimaryEventCallbackRegister(MCPWM_CALLBACK callback, uintptr_t context)
{
    mcpwmPriEventObj.callback = callback;
    mcpwmPriEventObj.context = context;
}



