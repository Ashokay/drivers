#ifndef DRIVER_PWM_H
#define	DRIVER_PWM_H

#ifdef	__cplusplus
extern "C" {
#endif
    void PWM_init(float *, float *); //calculates dt and PWM period values
    //*dt
    
	void driver_PWM_variables_update(int);
    //not used?
	
    void duty_cal_update(float, float, float);   
    //du, dv, dw
    
    void PWM_override_OFF(void);
    
    void PWMpins_enable(void); //remove override
    
    void PWM_bootstrap(void); //not used

#ifdef	__cplusplus
}
#endif

#endif	