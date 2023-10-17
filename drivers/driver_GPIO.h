#ifndef DRIVER_GPIO_H
#define	DRIVER_GPIO_H

#ifdef	__cplusplus
extern "C" {
#endif

    void turn_on_RED_LED(void);
    void turn_off_RED_LED(void);
    void RED_LED_blink(int); //no. of blinks
    void system_state_indicator(int, int);
    
    void GPIO_Vmode(float , float, float, float *);
    //float rpm_max, float eco_rpm, float *VM_rpm_limit_adr)
        
    void GPIO_init(float);
    
    void GPIO_reverse(float , float , float , float *);
    //Iph_max, rpm_max, reverse_rpm, *VM_rpm_limit_adr 
    
    void GPIO_immob_EN(float, float *);
    //float Iph_max, float *VM_rpm_limit_adr
    
    void GPIO_immob_DIS(float *);
    //(float *VM_rpm_limit_adr)


#ifdef	__cplusplus
}
#endif

#endif	/* DRIVER_GPIO_H */

