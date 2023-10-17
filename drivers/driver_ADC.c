#include "device.h"
#include "driver_ADC.h"
#include "math.h"
    
	#define ISCALE 1000.0
	#define VSCALE 98
	#define TSCALE 30.0

	#define TOFFSET 12.0
	#define INPUT_SCALE 5.0

    #define RTJS    1.85
    #define RDSON   2.7
    #define TRDS    170.0
    #define NSP     2.0

	static int Iu_adc = 2048, Iw_adc = 2048, Idc_adc = 2048, Vdc_adc = 0;	//Direct ADC values, needed for checking faults
	static float Iu_off = 2048, Iw_off = 2048, Idc_off = 2048;				//Offsets in currents, to be caliberated using 'adc_calib()'
        
	static float fMRcos = 0, fMRsin = 0;									//Filtered MR sensor input for magnitude calaculation
    static float UMRcos, UMRsin;                                            //Need unit for speed calc
	static float fMRCS_mag = 0.8;											//MR sensor magnitude
	static float MRcos0, MRsin0; 											//MR sensor angle offset to match 'rotor zero'

	static float fTu = 0, fTv = 0, fTw = 0, fTm = 0;						//temperatures for fault checking

	static int Vfltp = 3000, Vfltn = 2000, Ifltp = 4000, Ifltn = 100, Idcfltp = 4000, Idcfltn = 100;	//Fault limits
	static float tflt = 110.0, tm_flt = 110.0, rpm_flt = 8000; 
    static float fIdc_adc = 2000, fVdc_adc = 2500;
    
    static int throttle_error = 0;
    static int poles = 48;
    static float abs_rpm = 0;
    
    static float throttle_zero = 0.9, throttle_on = 1.0, throttle_max = 4.0;
    static float brake_zero = 1.9, brake_on = 2.0, brake_max = 4.0;


void measurement_constants_update(float c0, float s0, int p, float throt_zero, float throt_max, float b_zero, float b_max)
{
	MRcos0 = c0;
	MRsin0 = s0;
	   
    poles = p;
	   
    throttle_zero = throt_zero;
    throttle_max = throt_max;
    throttle_on = throttle_zero + 0.1;
    
    brake_zero = b_zero;
    brake_max = b_max;
    brake_on = brake_zero + 0.1;
}

void fault_limits_update(float Vmax, float Vmin, float Idcflt, float Iflt, float Tflt, float speed_flt, float Tm_flt)
{
	Ifltp = 2048 + (int)( Iflt * 4096/ISCALE);			//calculate phase current fault limits
	Ifltn = 2048 - (int)( Iflt * 4096/ISCALE);	
    
    if(Ifltp > 3996) Ifltp = 3996;
    if(Ifltp < 2048) Ifltp = 2048;
    
    if(Ifltn < 100) Ifltn = 100;
    if(Ifltn > 2048) Ifltn = 2048;
	
	Idcfltp = 2048 + (int)( Idcflt * 4096/ISCALE);  	//calculate DC current fault limits
	Idcfltn = 2048 - (int)( Idcflt * 4096/ISCALE);	
    
    if(Idcfltp > 3996) Idcfltp = 3996;
    if(Idcfltp < 2048) Idcfltp = 2048;
    
    if(Idcfltn < 100) Idcfltn = 100;
    if(Idcfltn > 2048) Idcfltn = 2048;
    
	Vfltp = (int)(Vmax * 4096 / VSCALE);				//calculate voltage fault limits
    Vfltn = (int)(Vmin * 4096 / VSCALE);				//calculate voltage fault limits
    
    if(Vfltp > 3100) Vfltp = 3100;
    if(Vfltn < 1000) Vfltn = 1000;
	
    if(Vfltp < Vfltn) Vfltn = Vfltp - 200; 
	
	tflt = Tflt;										//Temperature fault limit
    tm_flt = Tm_flt;
    
    if(tflt > 120)  tflt = 120.0;
    if(tflt < 0)   tflt = 0;    
    
    rpm_flt = speed_flt; 
}


void ADC_calib()
{
    Iu_off = Iu_off + 0.01*((float)Iu_adc - Iu_off);	//ADC current measurement offsets: zero caliberation
    Iw_off = Iw_off + 0.01*((float)Iw_adc - Iw_off);  	
    Idc_off = Idc_off + 0.01*((float)Idc_adc - Idc_off); 
}


void fast_measurement_VI(float *Vdc, float *Idc, float *Iu, float *Iv, float *Iw)
{
	if(ADCDSTAT1bits.ARDY0) Vdc_adc = ADCDATA0;
    if(ADCDSTAT1bits.ARDY1) Iu_adc = ADCDATA1;
    if(ADCDSTAT1bits.ARDY2) Iw_adc = ADCDATA2; 
    if(ADCDSTAT1bits.ARDY3) Idc_adc = ADCDATA3;

	*Iu = (float)Iu_adc;
    *Iu = (*Iu - Iu_off)*(ISCALE / 4096.0);
    
    *Iw = (float)Iw_adc;
    *Iw = (*Iw - Iw_off)*(ISCALE / 4096.0);
    
    *Iv = - *Iu - *Iw;
     
    *Idc = (float)Idc_adc;
    *Idc = (*Idc - Idc_off)*(ISCALE / 4096.0);    
    
    fIdc_adc = fIdc_adc + 0.005*((float)Idc_adc - fIdc_adc);
    
    *Vdc = (float)Vdc_adc;  
    *Vdc = *Vdc * (VSCALE / 4096.0);
    
    fVdc_adc = fVdc_adc + 0.5*((float)Vdc_adc - fVdc_adc);
}

void fast_measurement_RP(float *cosMR, float *sinMR, int MRdirection)
{
	int MRcosN = 0, MRcosP = 0, MRsinN = 0, MRsinP = 0;
  
	float MRcos, MRsin;
    float cos5MR, sin5MR;

    float cos_r2, cos_r3, cos_r5;
    float sin_r2, sin_r3, sin_r5;
	
    if(ADCDSTAT1bits.ARDY12) MRcosP = ADCDATA12;
    if(ADCDSTAT1bits.ARDY13) MRsinP = ADCDATA13;
    if(ADCDSTAT1bits.ARDY14) MRcosN = ADCDATA14;
    if(ADCDSTAT1bits.ARDY15) MRsinN = ADCDATA15;
    
    //DAC2CONbits.DACDAT = MRcosN;
    //DAC1CONbits.DACDAT = MRsinN;
    
    MRcos = ((float)MRcosP - (float)MRcosN)/4096;   //convert to float
    MRsin = ((float)MRsinP - (float)MRsinN)/4096;
        
    if(MRdirection == 1) MRsin = -MRsin;        //for reversing sensor orientation: CW/CCW
   
    fMRcos = fMRcos + 0.8*(MRcos - fMRcos);			//filter
    fMRsin = fMRsin + 0.8*(MRsin - fMRsin);
        
    fMRCS_mag = sqrt(fMRcos*fMRcos + fMRsin*fMRsin);
    if(fMRCS_mag <= 0.1) fMRCS_mag = 0.1; 
    
    UMRcos = fMRcos/fMRCS_mag;    					//unit
    UMRsin = fMRsin/fMRCS_mag;    					//unit
    
    cos_r2 = UMRcos * UMRcos;						//frequency*5
    cos_r3 = cos_r2 * UMRcos;
    cos_r5 = cos_r3 * cos_r2;

    sin_r2 = UMRsin * UMRsin;
    sin_r3 = sin_r2 * UMRsin;
    sin_r5 = sin_r3 * sin_r2;

    cos5MR = 16.0*cos_r5 - 20.0*cos_r3 + 5.0*UMRcos;
    sin5MR = 16.0*sin_r5 - 20.0*sin_r3 + 5.0*UMRsin;

    cos5MR = 0.8165 * 1;//cos5MR;						//power invarient dq
    sin5MR = 0.8165 * 0;//sin5MR;
    
    *cosMR = cos5MR * MRcos0 + sin5MR * MRsin0; 	//angle offset
    *sinMR = - sin5MR * MRcos0 + cos5MR * MRsin0; 
    
    
}


void slow_measurement_T(float *Tesc_adr, float *Imosfet_adr, float *Tm_adr)
{
	int tu = 0, tv = 0, tw = 0, tm = 0;   
    float Tu, Tv, Tw, Tm, fTesc;
	
	if(ADCDSTAT1bits.ARDY9) tu = ADCDATA9;
    if(ADCDSTAT1bits.ARDY10) tv = ADCDATA10;
    if(ADCDSTAT1bits.ARDY8) tw = ADCDATA8;
    
    //NTCG203JH472JT1   (3J = B3450, R1 = 330)
    #define TDIV 30.0
    #define TADD 12.0
        
	Tu = ((float)tu)/TDIV + TADD;				//sensor type
	Tv = ((float)tv)/TDIV + TADD;
	Tw = ((float)tw)/TDIV + TADD;
    
    fTu = fTu + 0.01*(Tu - fTu);
    fTv = fTv + 0.01*(Tv - fTv);
    fTw = fTw + 0.01*(Tw - fTw);
    
    if(fTu > fTv)
    {
        if(fTu > fTw) fTesc = fTu;
        else fTesc = fTw;
    }
    else
    {
        if(fTv > fTw) fTesc = fTv;
        else fTesc = fTw;
    }
    *Tesc_adr = fTesc; 
    
    *Imosfet_adr = NSP * sqrt(fabs(2*1000*(TRDS - fTesc)/RDSON/RTJS));
    
    if(ADCDSTAT1bits.ARDY17) tm = ADCDATA17;
    Tm = (3700 - (float)tm)/17.0;    //NTC JAST, R = 4k7
    
    fTm = fTm + 0.01*(Tm - fTm);    
    
    //DAC2CONbits.DACDAT = (int)(fTm/150.0*4096.0);
    //DAC1CONbits.DACDAT = (int)(fTm/150.0*4096.0);
    
    *Tm_adr = fTm;
}

void slow_measurement_input(float sdt, float *adr_wSensor, float * a_input_adr, float *b_input_adr)
{
	int Throttle_ADC = 0;
    static float fThrottle = 0.9;
    
    int brake_ADC = 0;
    static float fbrake = 0.9;
    
    	
	static float prev_cos = 0, prev_sin = 0;
	static float wMech = 0, fwMech = 0;
	float dcos, dsin;	
	
	dcos = (UMRcos - prev_cos)/(sdt);
    dsin = (UMRsin - prev_sin)/(sdt);
    
    wMech = UMRsin*dcos - UMRcos*dsin; 
    
    fwMech = fwMech + 0.05*(wMech - fwMech);
    
    prev_cos = UMRcos;
    prev_sin = UMRsin;
    
    *adr_wSensor = 5.0*fwMech;
	
    abs_rpm = fabs(*adr_wSensor/2/3.14159265*120/poles);
    
	if(ADCDSTAT1bits.ARDY19) Throttle_ADC = ADCDATA19; 
    
	fThrottle = fThrottle + 0.01*((float)(Throttle_ADC) * INPUT_SCALE / 4096.0 - fThrottle);
    
    *a_input_adr = (fThrottle - throttle_on)/(throttle_max - throttle_on);
    if(*a_input_adr < 0) *a_input_adr = 0;
    if(*a_input_adr > 1) *a_input_adr = 1; 
    
    DAC1CONbits.DACDAT = (int)(*a_input_adr*4096.0);
        
	if(ADCDSTAT1bits.ARDY18) brake_ADC = ADCDATA18; 
	fbrake = fbrake + 0.01*((float)(brake_ADC) * INPUT_SCALE / 4096.0 - fbrake);
    
    *b_input_adr = (fbrake - brake_on)/(brake_max - brake_on);
    if(*b_input_adr < 0) *b_input_adr = 0;
    if(*b_input_adr > 1) *b_input_adr = 1;    
}

int fast_fault_check(void)
{
	int fault_id = 0;
	
	if(fIdc_adc > Idcfltp) {fault_id = 3;}
    if(fIdc_adc < Idcfltn) {fault_id = 3;}
	
    if(Vdc_adc > Vfltp) {fault_id = 1;}
    if(fVdc_adc < Vfltn) {fault_id = 5;}
    
    if(Iu_adc > Ifltp) {fault_id = 2;}
    if(Iw_adc > Ifltp) {fault_id = 2;}
    if(Iu_adc < Ifltn) {fault_id = 2;}
    if(Iw_adc < Ifltn) {fault_id = 2;}
	
	return fault_id;
}

int slow_fault_check(void)
{
	int fault_id = 0;
	
	if(fTu > tflt) {fault_id = 4;}
    if(fTv > tflt) {fault_id = 4;}
    if(fTw > tflt) {fault_id = 4;}
    
    if(fTm > tm_flt) {fault_id = 6;}
	
    if(throttle_error != 0) fault_id = 9; 
    
    if(abs_rpm > rpm_flt) fault_id = 10;
	
	return fault_id;
}


