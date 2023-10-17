
#ifndef DRIVER_ADC_H
#define	DRIVER_ADC_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


void measurement_constants_update(float, float, int, float, float, float, float); 
//MRcos0, MRsin0, poles, throttle_zero, throttle_max

void fault_limits_update(float, float, float, float, float, float, float);
//fault_limits_update(float Vmax, float Vmin, float Idcflt, float Iflt, float Tflt, float speed_flt)


void fast_measurement_VI(float*, float *, float *, float *, float *);
//Vdc, Idc, Iu, Iv, Iw

void fast_measurement_RP(float *, float *, int);
//cosMR, sinMR

void slow_measurement_T(float *, float *, float *);
//Tesc, Imosfet, Tm

void slow_measurement_input(float, float *, float *, float *);
//Throttle

int fast_fault_check(void);
int slow_fault_check(void);

void ADC_calib();

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */