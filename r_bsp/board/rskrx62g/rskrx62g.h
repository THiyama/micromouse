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
* File Name    : rskrx62g.h
* H/W Platform : RSKRX63G
* Description  : Board specific definitions for the RSKRX62T and RSKRX63G.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 17.11.2011 1.00     First Release
*         : 29.08.2012          Updated H/W platform to include 62G
***********************************************************************************************************************/

#ifndef RSKRX62G_H
#define RSKRX62G_H

/* Local defines */
#define LED_ON              (0)
#define LED_OFF             (1)
#define SET_BIT_HIGH        (1)
#define SET_BIT_LOW         (0)
#define SET_BYTE_HIGH       (0xFF)
#define SET_BYTE_LOW        (0x00)

/* Switches */
#define SW_ACTIVE           0
#define SW1                 PORTE.PORT.BIT.B5
#define SW2                 PORTE.PORT.BIT.B4
#define SW3                 PORTB.PORT.BIT.B4
#define SW1_DDR             PORTE.DDR.BIT.B5
#define SW2_DDR             PORTE.DDR.BIT.B4
#define SW3_DDR             PORTB.DDR.BIT.B4
#define SW1_ICR             PORTE.ICR.BIT.B5
#define SW2_ICR             PORTE.ICR.BIT.B4
#define SW3_ICR             PORTB.ICR.BIT.B4

/* LEDs */
#define LED0                PORT7.DR.BIT.B1
#define LED1                PORT7.DR.BIT.B2
#define LED2                PORT7.DR.BIT.B3
#define LED3                PORT3.DR.BIT.B3
#define LED0_DDR            PORT7.DDR.BIT.B1
#define LED1_DDR            PORT7.DDR.BIT.B2
#define LED2_DDR            PORT7.DDR.BIT.B3
#define LED3_DDR            PORT3.DDR.BIT.B3

#endif /* RSKRX62G_H */
