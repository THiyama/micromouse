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
* File Name    : r_bsp.h
* H/W Platform : RSKRX62T
* Description  : Has the header files that should be included for this platform.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 13.01.2012 1.00     First Release
*         : 12.03.2012 1.10     Added locking.h and r_bsp_config.h #includes. Removed sbrk.h since heap size is not
*                               defined in r_bsp_config.h.
*         : 27.06.2012 1.20     Relocated mcu_info.h to the 'mcu' folder.
*         : 16.07.2012 1.30     Added vecttbl.h include. 
*         : 24.09.2012 1.40     Added test for C99 compliant compiler. Updated includes for v1.0 spec.
*         : 17.01.2013 1.50     Added #include for lowsrc.h.
***********************************************************************************************************************/

#ifndef BSP_BOARD_RSKRX62T
#define BSP_BOARD_RSKRX62T

/* Make sure that no other platforms have already been defined. Do not touch this! */
#ifdef  PLATFORM_DEFINED
#error  "Error - Multiple platforms defined in platform.h!"
#else
#define PLATFORM_DEFINED
#endif

/***********************************************************************************************************************
INCLUDE APPROPRIATE MCU AND BOARD FILES
***********************************************************************************************************************/
#if __STDC_VERSION__ >= 199901L                  
#include    <stdint.h>
#include    <stdbool.h>
#else
#include    "mcu/all/r_typedefs.h"
#endif

#include    "r_bsp_config.h"
#include    "mcu/rx62t/register_access/iodefine.h"
#include    "mcu/rx62t/mcu_info.h"
#include    "mcu/rx62t/mcu_locks.h"
#include    "mcu/rx62t/locking.h"
#include    "board/rskrx62t/rskrx62t.h"
#include    "board/rskrx62t/hwsetup.h"
#include    "board/rskrx62t/lowsrc.h"
#include    "board/rskrx62t/vecttbl.h"

#endif /* BSP_BOARD_RSKRX62T */


