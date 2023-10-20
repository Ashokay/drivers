#include "driver_GPIO.h"
#include "device.h"
#include "VCU_CANdata.h"

#define RED_LED LATCbits.LATC12
#define GREEN_LED LATBbits.LATB7
#define RED_LED_STATE PORTCbits.RC12
#define H_MODE_PIN PORTBbits.RB4
#define REVERSE_MODE_PIN PORTBbits.RB2 //PGC pin
#define IMMOB_MODE_PIN PORTBbits.RB3  //PGD pin
#define L_MODE_PIN PORTBbits.RB5  

int GPIO_reverse() 
{
    static int reverse_EN = 0;
    static int reverse_pin_state = 1;
    
    if(REVERSE_MODE_PIN != reverse_pin_state)
    {
        reverse_pin_state = REVERSE_MODE_PIN;
        
        if(REVERSE_MODE_PIN == 0)  reverse_EN = 1;
        else reverse_EN = 0;        
    }
    reverse_indicator(reverse_EN);    
    return(reverse_EN);
}

int GPIO_immob()
{       
    static int immob_EN = 0;
    static int immob_pin_state = 1;
    
    if(IMMOB_MODE_PIN != immob_pin_state)
    {
        immob_pin_state = IMMOB_MODE_PIN;
        
        if(IMMOB_MODE_PIN == 0) immob_EN = 1;        
        else immob_EN = 0;
    }
    side_stand_indicator(immob_EN);
    return(immob_EN);
}

int GPIO_Vmode()
{
    static int Vmode = 0;
    static int Hmode_pin_state = 1;
    static int Lmode_pin_state = 1;
    
    if((H_MODE_PIN != Hmode_pin_state)|| (L_MODE_PIN != Lmode_pin_state))   
    {
        Hmode_pin_state = H_MODE_PIN;
        Lmode_pin_state = L_MODE_PIN;
        
        if((Hmode_pin_state == 1) & (Lmode_pin_state == 1)) 
        {
            Vmode = 0;
            Lmode_indicator(0);
            Hmode_indicator(0);
        }
        else if(Lmode_pin_state == 0) 
        {               
            
            Vmode = 1;
            Lmode_indicator(1);
            Hmode_indicator(0);
        }
        else
        {                 
            Vmode = 2;
            Lmode_indicator(0);
            Hmode_indicator(1);
        }
    } 
    return Vmode;
}

void turn_on_RED_LED(void)
{
    RED_LED = 1;
}

void turn_off_RED_LED(void)
{
    RED_LED = 0;
}
    
void RED_LED_blink(int blink_value)
{
    #define LED_BLINK_ON_TIME 1500         
    #define LED_BLINK_OFF_TIME 1500
    
    static long int LED_time = 0;
    static int LED_blink_count = 0;   
    
    LED_time++;
    if(RED_LED_STATE == 1)
    {
        if(LED_time > LED_BLINK_ON_TIME)
        {
            LED_time = 0;
            LED_blink_count++;
            RED_LED = 0;
        }
    }
    else
    {
        if(LED_blink_count < blink_value)
        {
            if(LED_time > LED_BLINK_OFF_TIME)
            {
                LED_time = 0;
                RED_LED = 1;
            }
        }
        else
        {
            if(LED_time > LED_BLINK_OFF_TIME*5)
            {
                LED_time = 0;
                LED_blink_count = 0;
            }            
        }        
    } 
}

void system_state_indicator(int sys_state, int faultno)
{
    GREEN_LED = 1;
    if(sys_state == 0) RED_LED_blink(faultno);
    else if(sys_state == 2) turn_on_RED_LED();
    else turn_off_RED_LED();
}
