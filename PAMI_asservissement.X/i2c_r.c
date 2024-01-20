/**
  I2C_R Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c_r.c

  @Summary
    This is the generated header file for the i2c_r driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for i2c_r.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.4
        Device            :  dsPIC33CK256MP502

    The generated drivers are tested against the following:
        Compiler          :  XC16 v2.10
        MPLAB             :  MPLAB X v6.05
*/

/* 
 * File:   i2c_r.c
 * Author: axeltremaudant
 *
 * Created on December 8, 2023, 11:12 AM
 */

/*
 * The code has been designed to transmit the rotating speed of the motor on one byte.
 * To allow that, the first bit is a sign bit (0 is positive, 1 is negative),
 * the 7 next bits represent the absolute rotating speed divided by a coefficient.
 */

#include <stdio.h>
#include <stdlib.h>

#include "i2c_r.h"

/**
 Section: Data Types
*/

/**
  I2C Slave Driver State Enumeration

  @Summary
    Defines the different states of the i2c slave.

  @Description
    This defines the different states that the i2c slave
    used to process transactions on the i2c bus.
*/
typedef enum
{
    S_SLAVE_IDLE,
    S_SLAVE_RECEIVE_MODE,
    S_SLAVE_TRANSMIT_MODE,
} I2C_R_SLAVE_STATES;


/**
 Section: Macro Definitions
*/
/* defined for I2C2 */
#define I2C_R_TRANSMIT_REG                       I2C2TRN	// Defines the transmit register used to send data.
#define I2C_R_RECEIVE_REG                        I2C2RCV	// Defines the receive register used to receive data.

#define I2C_R_MASK_REG                           I2C2MSK	// Defines the address mask register.
#define I2C_R_ADDRESS_REG                        I2C2ADD	// Defines the address register. 

// The following control bits are used in the I2C state machine to manage
// the I2C module and determine next states.
#define I2C_R_GENERAL_CALL_ENABLE_BIT            I2C2CONLbits.GCEN	// I2C General Call enable control bit.
#define I2C_R_RELEASE_SCL_CLOCK_CONTROL_BIT      I2C2CONLbits.SCLREL	// I2C clock stretch/release control bit.

// The following status bits are used in the I2C state machine to determine
// the next states.

#define I2C_R_READ_NOT_WRITE_STATUS_BIT          I2C2STATbits.R_W    // I2C current transaction read/write status bit.
#define I2C_R_DATA_NOT_ADDRESS_STATUS_BIT        I2C2STATbits.D_A    // I2C last byte receive was data/address status bit.
#define I2C_R_RECEIVE_OVERFLOW_STATUS_BIT        I2C2STATbits.I2COV	// I2C receive buffer overflow status bit.
#define I2C_R_GENERAL_CALL_ADDRESS_STATUS_BIT    I2C2STATbits.GCSTAT	// I2C General Call status bit.
#define I2C_R_ACKNOWLEDGE_STATUS_BIT             I2C2STATbits.ACKSTAT	// I2C ACK status bit.

/**
 Section: Local Functions
*/

static inline void __attribute__ ((always_inline)) I2C_R_TransmitProcess(void);
static inline void __attribute__ ((always_inline)) I2C_R_ReceiveProcess(void);
static inline void __attribute__ ((always_inline)) toggle_led_receive_r(void);
static inline void __attribute__ ((always_inline)) toggle_led_transmit_r(void);
void (*I2C_R_ReceiveHandler)(int) = NULL;

/**
 Section: Local Variables
*/

static I2C_R_SLAVE_STATES   i2c_r_slave_state;
static uint8_t            i2c_r_write;
static uint8_t            *p_read_speed_nb_measure_r;
static int                *p_read_speed_sum_r;

/**
  Prototype:        void I2C_R_Initialize(void)
  Input:            none
  Output:           none
  Description:      I2C_R_Initialize is an
                    initialization routine that takes inputs from the GUI.
  Comment:          
  Usage:            I2C_R_Initialize();
*/
void I2C_R_Initialize(const uint16_t address)
{

    // initialize the hardware
    // ACKEN disabled; STREN enabled; GCEN disabled; SMEN disabled; DISSLW enabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; IPMIEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled; 
    I2C2CONL = 0x8040;
    // BCL disabled; P disabled; S disabled; I2COV disabled; IWCOL enabled; 
    I2C2STAT = 0x80;
    // I2CADD 87; 
    I2C_R_SlaveAddressSet(address);
    // AMSK 0; 
    I2C_R_SlaveAddressMaskSet(0x00);

    // make sure this is set first
    i2c_r_slave_state = S_SLAVE_IDLE;
    
    I2C_R_ReadPointerSet(NULL, NULL);
    i2c_r_write = 0;
    
    // INTERRUPTS
    //    MICI: I2C2 Master Events
    //    Priority: 7
    IPC9bits.MI2C2IP = 7;
    //    SICI: I2C2 Slave Events
    //    Priority: 7
    IPC9bits.SI2C2IP = 7;
    
    /* I2C1 Slave Events */
    // clear the master interrupt flag
    IFS2bits.SI2C2IF = 0;
    // enable the master interrupt
    IEC2bits.SI2C2IE = 1;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _SI2C2Interrupt ( void )
{
    uint8_t dummy;

    // NOTE: The slave driver will always acknowledge 
    //       any address match.


    switch (i2c_r_slave_state)
    {
        case S_SLAVE_IDLE:
        case S_SLAVE_RECEIVE_MODE:

            /* When at S_SLAVE_RECEIVE_MODE this mode there
               will be two types of incoming transactions:
               1. Data sent by master
               2. A restart or start detection

               But from the point of view of the firmware, there is
               no difference between S_SLAVE_IDLE and S_SLAVE_RECEIVE_MODE
               states, since the types of incoming transactions will be
               the same so we share the code here.
             */           
                            
            if (I2C_R_READ_NOT_WRITE_STATUS_BIT == 0)
            {
                // it is a write, go to receive mode 

                // Receive the data if valid
                I2C_R_ReceiveProcess(); // Get address of communication
                i2c_r_slave_state = S_SLAVE_RECEIVE_MODE;
            }
            else
            {
                // read the receive register only when
                // we are ready for the next transaction.
                // this one is a dummy read
                dummy = I2C_R_RECEIVE_REG;

                // it is a read, go to transmit mode

                // during this portion, the master is expecting the
                // slave for a reply. So the returned status of
                // the callback at this point cannot be used to 
                // delay the reply if needed.
                // In other words, the slave has to reply to the master.
                // Therefore, the transmit will be performed.

                I2C_R_TransmitProcess();

                i2c_r_slave_state = S_SLAVE_TRANSMIT_MODE;
            }

            // this if statement is to make sure we only save incoming
            // data when we are truly in receiving mode
            if (i2c_r_slave_state == S_SLAVE_RECEIVE_MODE)
            {
                // case of data received
                if (I2C_R_DATA_NOT_ADDRESS_STATUS_BIT == 1)
                {
                    // check if we are overflowing the receive buffer
                    if (I2C_R_RECEIVE_OVERFLOW_STATUS_BIT != 1)
                    {
                        I2C_R_ReceiveProcess();
                        
                        int value = i2c_r_write & 0b01111111; // Get the 7-bits value
                        if(i2c_r_write & 0b10000000) // Get the bit sign
                            value = -value;
                        
                        I2C_R_ReceiveHandler(value);
                    }
                    else
                    {
                        // overflow detected!
                        // read the buffer to reset the buffer full flag
                        // and clear the overflow bit
                        // then do nothing so the master
                        // will resend the data
                        dummy = I2C_R_RECEIVE_REG;
                        I2C_R_RECEIVE_OVERFLOW_STATUS_BIT = 0;
                    }
                }
            }

            break;

        case S_SLAVE_TRANSMIT_MODE:

            // this is the state where an ACK or NACK is expected
            // to occur after the slave has placed data to the
            // transmit register.

            // if the transaction was ACK'ed, more data needs to be sent
            // if the transaction was NACK'ed then we don't need to send
            // more data
            if (I2C_R_ACKNOWLEDGE_STATUS_BIT == 0)
            {
                // transmit more data
                I2C_R_TransmitProcess();
            }
            else //if (I2C_R_ACKNOWLEDGE_STATUS_BIT == 1)
            {
                // no more data to be sent so we go to idle state
                i2c_r_slave_state = S_SLAVE_IDLE;
            }
            break;


        default:
            // should never happen, if we ever get here stay here forever
            while(1);
            break;
    }

    I2C_R_RELEASE_SCL_CLOCK_CONTROL_BIT = 1;

    // clear the slave interrupt flag
    IFS1bits.SI2C1IF = 0;
}

void I2C_R_ReadPointerSet(int *sum_speed, uint8_t *measure_count)
{
    p_read_speed_nb_measure_r = measure_count;
    p_read_speed_sum_r = sum_speed;
}

void I2C_R_SlaveAddressMaskSet(uint16_t mask)
{
    I2C_R_MASK_REG = mask;
}

void I2C_R_SlaveAddressSet(uint16_t address)
{
    i2c_r_slave_state = S_SLAVE_IDLE;
    I2C_R_ADDRESS_REG = address;
}

static inline void __attribute__ ((always_inline)) I2C_R_TransmitProcess(void)
{
    // get the data to be transmitted
    
    // Calculate the average speed reduced
    int val = *p_read_speed_sum_r / *p_read_speed_nb_measure_r;
    
    // Set the message to be sent
    uint8_t message = val < 0 ? 0b10000000 + (uint8_t) -val : (uint8_t) val;
    
    I2C_R_TRANSMIT_REG = message;
    
    // Reset the values
    *p_read_speed_sum_r = *p_read_speed_nb_measure_r = 0;

    // set the SCL clock to be released
    I2C_R_RELEASE_SCL_CLOCK_CONTROL_BIT = 1;
    toggle_led_transmit_r();
}

static inline void __attribute__ ((always_inline)) I2C_R_ReceiveProcess(void)
{   
    // store the received data 

    i2c_r_write = I2C_R_RECEIVE_REG;
    toggle_led_receive_r();
}

static inline void __attribute__ ((always_inline)) toggle_led_transmit_r()
{
    //_LATB4 = !_LATB4;
}

static inline void __attribute__ ((always_inline)) toggle_led_receive_r()
{
    //_LATA4 = !_LATA4;
}

void I2C_R_set_receive_handler(void (* interruptHandler)(int))
{
    I2C_R_ReceiveHandler = interruptHandler;
}

