#include "driver_GPIO.h"
#include "device.h"

#define RED_LED LATCbits.LATC12
#define RED_LED_STATE PORTCbits.RC12
#define ECO_MODE_PIN PORTBbits.RB4
#define REVERSE_MODE_PIN PORTBbits.RB5
#define IMMOB_MODE_PIN PORTBbits.RB3  //PGD pin
#define LECO_MODE_PIN PORTBbits.RB2  //PGD pin

static float VM_rpm_limit_mem = 0;
static int immob_EN = 0;
static int reverse_EN = 0;

void GPIO_init(float rpm_max)
{
    VM_rpm_limit_mem = rpm_max;
}

//called only in drive reset mode (vehicle is stationary)   
void GPIO_reverse(float Iph_max, float rpm_max, float reverse_rpm, float *VM_rpm_limit_adr) 
{
   
    if(REVERSE_MODE_PIN)
    {
        reverse_EN = 1;
                   
        if(*VM_rpm_limit_adr >= 1) VM_rpm_limit_mem = *VM_rpm_limit_adr;       //store mode speed
        *VM_rpm_limit_adr = -reverse_rpm * rpm_max/100;        
    }
    else
    {
        if(reverse_EN == 1)
        {
            *VM_rpm_limit_adr = VM_rpm_limit_mem;
            reverse_EN = 0;
        }
    }
}

//called only in drive reset mode (vehicle is stationary)
void GPIO_immob_EN(float Iph_max, float *VM_rpm_limit_adr)
{       
    if(IMMOB_MODE_PIN == 1)
    {
        immob_EN = 1;
        
        if(*VM_rpm_limit_adr >= 1) VM_rpm_limit_mem = *VM_rpm_limit_adr; 
        *VM_rpm_limit_adr = 0;
    }
    else
    {
        if(immob_EN == 1)
        {
            *VM_rpm_limit_adr = VM_rpm_limit_mem;    //here throttle will be zero = safe start
            immob_EN = 0;
        }
    }
}

void GPIO_immob_DIS(float *VM_rpm_limit_adr)
{
    if(immob_EN == 1)
    {
        if(IMMOB_MODE_PIN == 0)
        {
            *VM_rpm_limit_adr = 0.1;  //release immob to reset the drive safely (as throttle could be high).
        }
    }
}

void GPIO_Vmode(float rpm_max, float eco_rpm, float leco_rpm, float *VM_rpm_limit_adr)
{
    static int eco_pin_state = 0;
    
    if((ECO_MODE_PIN != eco_pin_state) & (*VM_rpm_limit_adr >= 1))   //VM_rpm_limit < 0 = reverse, VM_rpm_limit = 0 = immob 0.1 = immob release
    {
        eco_pin_state = ECO_MODE_PIN;
        if(eco_pin_state == 1) 
        {
            *VM_rpm_limit_adr = eco_rpm*rpm_max/100.0;
            if(*VM_rpm_limit_adr < 1) *VM_rpm_limit_adr = 1;            
        }
        else
        {                
            *VM_rpm_limit_adr = rpm_max;
        }
    }
    
    static int leco_pin_state = 0;
    
    if((LECO_MODE_PIN != leco_pin_state) & (*VM_rpm_limit_adr >= 1))   //VM_rpm_limit < 0 = reverse, VM_rpm_limit = 0 = immob 0.1 = immob release
    {
        leco_pin_state = LECO_MODE_PIN;
        if(leco_pin_state == 1) 
        {
            *VM_rpm_limit_adr = leco_rpm*rpm_max/100.0;
            if(*VM_rpm_limit_adr < 1) *VM_rpm_limit_adr = 1;            
        }
        else
        {                
            *VM_rpm_limit_adr = rpm_max;
        }
    } 
    
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
    if(sys_state == 0) RED_LED_blink(faultno);
    else if(sys_state == 2) turn_on_RED_LED();
    else turn_off_RED_LED();
}
