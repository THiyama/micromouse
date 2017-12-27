/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No 
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all 
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM 
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES 
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS 
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of 
* this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer 
*
* Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.    
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : hwsetup.c
* Device(s)    : RX
* H/W Platform : YRDKRX62N
* Description  : Defines the initialization routines used each time the MCU is restarted.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 26.10.2011 1.00     First Release
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
/* I/O Register and board definitions */
#include "platform.h"
/* Contains delcarations for the functions defined in this file */
#include "hwsetup.h"

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/
/* MCU I/O port configuration function delcaration */
static void output_ports_configure(void);

/* Interrupt configuration function delcaration */
static void interrupts_configure(void);

/* MCU peripheral module configuration function declaration */
static void peripheral_modules_enable(void);


/***********************************************************************************************************************
* Function name: hardware_setup
* Description  : Contains setup functions called at device restart
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
void hardware_setup(void)
{
    output_ports_configure();
    interrupts_configure();
    peripheral_modules_enable();
}

/***********************************************************************************************************************
* Function name: output_ports_configure
* Description  : Configures the port and pin direction settings, and sets the pin outputs to a safe level.
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
static void output_ports_configure(void)
{
    /* Port 0 - all inputs (IRQ's from ethernet & WiFi) */
    PORT0.DDR.BYTE = 0x00;
    
    /* Port 1 - all inputs (IIC and USB settings will override these later */
    PORT1.DDR.BYTE = 0x00;

    /* Port 2 - USB */
    PORT2.DR.BYTE = 0x00;   /* All outputs low to start */
    PORT2.DDR.BIT.B3 = 1;   /* USB0 DPUPEA is an output */
    PORT2.DDR.BIT.B4 = 1;   /* USB VBUSEN is an output */

    /* Port 3 - JTAG (P30, P31, P34), CAN (P32=Tx, P33=Rx), NMI (P35) */
    PORT3.DR.BIT.B2 = 0;    /* Tx low to start */
    PORT3.DDR.BIT.B2 = 1;   /* Tx is an output */

    /* Port 4 - Switches (P40-P42), AIN (P43-P47)  */
    PORT4.DDR.BYTE = 0x00;  /* Port 4 is all inputs */
    PORT4.ICR.BYTE = 0xFF;  /* Turn on input buffers */

    /* Port 5 - LCD control signals, some audio  */
    PORT5.DR.BYTE = 0x00;   /* All outputs off to start */           
    PORT5.DDR.BIT.B0 = 1;   /* TxD2 is output. */
    PORT5.DDR.BIT.B1 = 1;   /* LCD_RS is an output */
    PORT5.DDR.BIT.B2 = 0;   /* RxD2 is input */
    PORT5.ICR.BIT.B2 = 1;   /* Enable input buffer on RxD2. */
    PORT5.DDR.BIT.B3 = 1;   /* BCLK is an output */
    PORT5.DDR.BIT.B4 = 1;   /* PWMLP_OUT is an output */
    PORT5.DDR.BIT.B5 = 1;   /* AMP_SHDN is an output */
    
    /* SCI2 B pins are used. */
    IOPORT.PFFSCI.BYTE = 0x04;
    
    /* Port A outputs all LOW to start  */
    PORTA.DR.BYTE = 0x00;           

    /* Port A - Expansion (PA0-PA2), Ether (PA3-PA5), Audio (PA6-PA7) */
    PORTA.DDR.BYTE = 0x00;  /* All pins are inputs for now */

    /* Port B outputs all LOW to start  */
    PORTB.DR.BYTE = 0x00;

    /* Port B - Ether */
    PORTB.DDR.BIT.B4 = 1;   /* RMII_TX_EN is an output */
    PORTB.DDR.BIT.B5 = 1;   /* RMII_TXD0 is an output */
    PORTB.DDR.BIT.B6 = 1;   /* RMII_TXD6 is an output */

    /* Choose A pin group and enable RSPIA pins */    
    IOPORT.PFGSPI.BYTE = 0x0E;
    
    /* Port C - Chip selects, clock = high; IO reset = low */
    PORTC.DR.BYTE = 0x17;

    /* Port C - SPI (PC0-2, PC4-7), IO reset (PC3) */
    PORTC.DDR.BIT.B0 = 1;   /* FLASH_CS  is an output */
    PORTC.DDR.BIT.B1 = 1;   /* WIFI_CS is an output */
    PORTC.DDR.BIT.B2 = 1;   /* LCD_CS is an output */
    PORTC.DDR.BIT.B3 = 1;   /* LCD_RESET is an output */
    PORTC.DDR.BIT.B4 = 1;   /* SDMICRO_CS is an output */
    PORTC.DDR.BIT.B5 = 1;   /* RSPI_SCLOCK is an output */
    PORTC.DDR.BIT.B6 = 1;   /* RSPI_MOSI is an output */
    PORTC.DDR.BIT.B7 = 0;   /* MTCLKB-B/RSPI_MISO is an input */
    PORTC.ICR.BIT.B7 = 1;
    
    /* Move MTCLKB */
    IOPORT.PFCMTU.BYTE = 0x80;

    /* Port D - All LED's off */
    PORTD.DR.BYTE = 0xFF;

    /* Port D - LED's */
    PORTD.DDR.BYTE = 0xFF;  /* All are outputs */           

    /* Port E - All LED's off */
    PORTE.DR.BYTE = 0x0F;

    /* Port E - LED's */
    PORTE.DDR.BYTE = 0x0F;

}

/***********************************************************************************************************************
* Function name: interrupts_configure
* Description  : Configures interrupts used
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
static void interrupts_configure(void)
{
    /* Add code here to setup additional interrupts */
}

/***********************************************************************************************************************
* Function name: peripheral_modules_enable
* Description  : Enables and configures peripheral devices on the MCU
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
static void peripheral_modules_enable(void)
{
    /* Add code here to enable peripherals used by the application */
}
