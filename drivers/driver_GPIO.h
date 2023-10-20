#ifndef DRIVER_GPIO_H
#define	DRIVER_GPIO_H

#ifdef	__cplusplus
extern "C" {
#endif

    void turn_on_RED_LED(void);
    void turn_off_RED_LED(void);
    void RED_LED_blink(int); //no. of blinks
    void system_state_indicator(int, int);
    
    int GPIO_Vmode();
    
    int GPIO_reverse();
    
    int GPIO_immob();
    

#ifdef	__cplusplus
}
#endif

#endif	/* DRIVER_GPIO_H */

