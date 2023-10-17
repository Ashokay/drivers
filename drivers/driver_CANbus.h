

#ifndef _DRIVER_CANBUS_H    /* Guard against multiple inclusion */
#define _DRIVER_CANBUS_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void CANbus_init(void);         //Initiialize pins
    
    void CANbus_write(uint8_t, uint8_t *, uint8_t);        //add arguments as per protocol requirement
    
    void CANbus_read(void);         //add arguments as per protocol requirements


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CANBUS */

/* *****************************************************************************
 End of File
 */
