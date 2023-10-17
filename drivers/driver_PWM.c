#include "driver_PWM.h"
#include "device.h"
	
    #define DT 0.000040
    #define PWMPER 2400
    #define PWMPERH 1200

    #define DUTY_SATURATION 0.95

    static float half_pwm_period = 1200.0;
 
    
void PWM_init(float * adr_dt, float * adr_duty_sat)
{
    PTPER = PWMPER;
    SEVTCMP = PWMPERH;
    
    half_pwm_period = PWMPERH;
    
    PDC1 = (int)half_pwm_period;
    PDC2 = (int)half_pwm_period;
    PDC3 = (int)half_pwm_period;
    
    *adr_dt = DT;        
    *adr_duty_sat = DUTY_SATURATION;
}
    

void PWM_override_OFF(void)
{
    IOCON3bits.OVRDAT   = 0;            //Override data 0 (stop)
    IOCON3bits.OVRENH   = 1;            //Override to stop 
    IOCON3bits.OVRENL   = 1;            //Override to stop
    
    IOCON2bits.OVRDAT   = 0;            //Override data 0 (stop)
    IOCON2bits.OVRENH   = 1;            //Override to stop 
    IOCON2bits.OVRENL   = 1;            //Override to stop
    
    IOCON1bits.OVRDAT   = 0;            //Override data 0 (stop)
    IOCON1bits.OVRENH   = 1;            //Override to stop 
    IOCON1bits.OVRENL   = 1;            //Override to stop
}

void PWM_bootstrap(void)
{
    IOCON3bits.OVRDAT   = 1;            //Override data 0 (stop)
    IOCON3bits.OVRENH   = 1;            //Override to stop 
    IOCON3bits.OVRENL   = 1;            //Override to stop
    
    IOCON2bits.OVRDAT   = 1;            //Override data 0 (stop)
    IOCON2bits.OVRENH   = 1;            //Override to stop 
    IOCON2bits.OVRENL   = 1;            //Override to stop
    
    IOCON1bits.OVRDAT   = 1;            //Override data 0 (stop)
    IOCON1bits.OVRENH   = 1;            //Override to stop 
    IOCON1bits.OVRENL   = 1;            //Override to stop
}

void PWMpins_enable(void)
{
    IOCON3bits.OVRENH   = 0;           
    IOCON3bits.OVRENL   = 0;           
    
    IOCON2bits.OVRENH   = 0;             
    IOCON2bits.OVRENL   = 0;           
    
    IOCON1bits.OVRENH   = 0;             
    IOCON1bits.OVRENL   = 0;            
}

void driver_PWM_variables_update(int PWM_period)
{
	half_pwm_period = (float)PWM_period * 0.5;
}

void duty_cal_update(float dr, float dy, float db)
{
    int tempI;    

    tempI = (int)((dr + 1) * half_pwm_period);
    PDC1 = tempI;

    tempI = (int)((dy + 1) * half_pwm_period);
    PDC2 = tempI;
    
    tempI = (int)((db + 1) * half_pwm_period);
    PDC3 = tempI;    
}