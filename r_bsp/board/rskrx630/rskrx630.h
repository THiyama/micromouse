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
* File Name    : rskrx630.h
* H/W Platform : RSKRX630
* Description  : Board specific definitions for the RSKRX630.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 22.11.2011 1.00     First Release
***********************************************************************************************************************/

#ifndef RSKRX630_H
#define RSKRX630_H

/* Local defines */
#define LED_ON              (0)
#define LED_OFF             (1)
#define SET_BIT_HIGH        (1)
#define SET_BIT_LOW         (0)
#define SET_BYTE_HIGH       (0xFF)
#define SET_BYTE_LOW        (0x00)

/* Switches */
#define SW_ACTIVE           0
#define SW1                 PORT3.PIDR.BIT.B2
#define SW2                 PORT4.PIDR.BIT.B4
#define SW3                 PORT0.PIDR.BIT.B7
#define SW1_PDR             PORT3.PDR.BIT.B2
#define SW2_PDR             PORT4.PDR.BIT.B4
#define SW3_PDR             PORT0.PDR.BIT.B7
#define SW1_PMR             PORT3.PMR.BIT.B2
#define SW2_PMR             PORT4.PMR.BIT.B4
#define SW3_PMR             PORT0.PMR.BIT.B7

/* LEDs */
#define LED0                PORTC.PODR.BIT.B5
#define LED1                PORT2.PODR.BIT.B4
#define LED2                PORTC.PODR.BIT.B2
#define LED3                PORT1.PODR.BIT.B7
#define LED0_PDR            PORTC.PDR.BIT.B5
#define LED1_PDR            PORT2.PDR.BIT.B4
#define LED2_PDR            PORTC.PDR.BIT.B2
#define LED3_PDR            PORT1.PDR.BIT.B7

#endif /* RSKRX630_H */
