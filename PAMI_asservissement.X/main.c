/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.4
        Device            :  dsPIC33CK256MP502
    The generated drivers are tested against the following:
        Compiler          :  XC16 v2.10
        MPLAB 	          :  MPLAB X v6.05
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include "i2c_r.h"
#include "i2c_l.h"
#include "motor_functions.h"
#include "mcc_generated_files/tmr1.h"

const uint8_t i2c_right_address = 0x53, i2c_left_address = 0x54;

/*
 * Initialisation of the PWM on pins 23 and 26
 */
void init_PWM()
{    
    /* Set PWM Period on Secondary Time Base */ 
    MPER = 3684; // 500 us
    
    // Set duty cycles
    PG1DC = 1842; // 50 %
    PG2DC = 921; // 25 % 

    /* Set PWM Mode to Independent */ 
    PG1IOCONL = 0;
    PG1IOCONH = 0x14; // Enable PWM1-L
    PG2IOCONL = 0;
    PG2IOCONH = 0x18; // Enable PWM2-H

    /* Enable PWM, Independaent Edge Mode and set Duty Cycle master */ 
    PG1CONH = PG2CONH = 0x6000;
    PG1CONL = PG2CONL = 0x8008;
}

/*
 * Initilisation of the QEI modules
 */
void init_QEI(void)
{
    RPINR14 = 0x2d2e; // Set QEI_R on RB13 and RB14 (pins 24 and 25)
    RPINR16 = 0x2a2b; // Set QEI_L on RB10 and RB11 (pins 21 and 22)
    POS1CNTL = 0;
    
    // Set parameters
    QEI1CONbits.CCM    = 0; // Counter Control Mode Selection bits set as x4 mode
    QEI1CONbits.INTDIV = 7; // Timer clock prescaler set as 1:128
    QEI1CONbits.IMV    = 0; // Index match value
    QEI1IOCbits.FLTREN = 0; // Deactivate filter
    QEI1CONbits.PIMOD  = 0; // Position counter is unaffected by the Index input
    QEI1IOCbits.SWPAB  = 0; // Don't swap QEA and QEB
    QEI1CONbits.QEIEN  = 1; // Enable QEI module
}

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    init_QEI();
    SYSTEM_Initialize();
    I2C_R_Initialize(i2c_right_address);
    I2C_L_Initialize(i2c_left_address);
    init_PWM();
    
    TMR1_SetInterruptHandler(&speed_rotation_measure);
    I2C_R_set_receive_handler(&set_rotating_speed_target_r);
    I2C_L_set_receive_handler(&set_rotating_speed_target_l);
    
    // A priori useless, set just in case, to be replaced if necessary
    int i2c_left_read = 'A';
    int i2c_right_read = 'B';
    uint8_t count_read = 1;
    I2C_R_ReadPointerSet(&i2c_right_read, &count_read);
    I2C_L_ReadPointerSet(&i2c_left_read, &count_read);
            
    TMR1_Start();
    
    while (1)
    {
        // Nothing to do, everything is handled by interrupts 
    }
    return 1; 
}
/**
 End of File
*/

