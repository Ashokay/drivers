#include "device.h"
#include "../V3_MCF1024/peripheral/spi/spi_master/plib_spi4_master.h"
#include "stdio.h"


#define THERM1 LATCbits.LATC8

 
 
void slow_measurement_MT(float * Tm_adr)
{
    volatile uint16_t Utm;
    volatile static uint16_t SPI_r;
    static int SPI_state = 0;
    static int CS_wait = 0;
    
    volatile static float tm = 0, ftm = 0;
    
    switch(SPI_state)
    {
        case 0:
            THERM1 = 0;             //Enable CS for read
            SPI_state = 1;            
        break;
        
        case 1:
            SPI4BUF = 0xFFFF;       //Send clock for 16 bits
            SPI_state = 2;
        break;
        
        case 2:            
            SPI_r =  SPI4BUF;       //Read the buffer
            SPI_r = SPI_r;
            Utm = SPI_r >> 3;
            tm = (float)Utm/4.0;
            
            ftm = ftm + 0.5*(tm - ftm);
            SPI_state = 3;
        break;
        
        case 3:
            //SPI4STATbits.SPIROV = 0;
            //SPI4STAT = 0;
            THERM1 = 1;
            CS_wait ++;
            if(CS_wait > 10000)
            {
                //SPI_r =  SPI4BUF; 
                CS_wait = 0;                    
                SPI_state = 4;
            }
            else
                SPI_state = 3;
        break;
                              

        
                
        default:
            SPI_state = 0;
        break;
    }
    
    *Tm_adr = ftm;
}
