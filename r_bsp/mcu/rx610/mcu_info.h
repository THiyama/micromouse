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
* File Name    : mcu_info.h
* Device(s)    : RX610
* Description  : Information about the MCU on this board (RSKRX610).
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 17.11.2011 1.00     First Release
*         : 12.03.2012 1.10     System clock speeds are now calculated from macros in r_bsp_config.h.
*         : 27.06.2012 1.20     MCU group, package, and memory sizes are now based off of info given in r_bsp_config.h.
*         : 19.11.2012 1.30     Added null argument macros. Updated code to use 'BSP_' and 'BSP_CFG_' prefix for macros.
*         : 18.01.2013 1.40     Added BSP_MCU_IPL_MAX and BSP_MCU_IPL_MIN macros.
***********************************************************************************************************************/

#ifndef MCU_INFO
#define MCU_INFO

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Gets MCU configuration information. */
#include "r_bsp_config.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/* MCU Series. */
#if   BSP_CFG_MCU_PART_SERIES == 0x0
    #define BSP_MCU_SERIES_RX600    (1)
#else
    #error "ERROR - BSP_CFG_MCU_PART_SERIES - Unknown MCU Series chosen in r_bsp_config.h"
#endif

/* This macro means that this MCU is part of the RX61x collection of MCUs (i.e. RX610). */
#define BSP_MCU_RX61_ALL            (1)

/* MCU Group name. */
#if   BSP_CFG_MCU_PART_GROUP == 0x0
    #define BSP_MCU_RX610           (1)
#else
    #error "ERROR - BSP_CFG_MCU_PART_GROUP - Unknown MCU Group chosen in r_bsp_config.h"
#endif

/* Package. */
#if   BSP_CFG_MCU_PART_PACKAGE == 0x0
    #define BSP_PACKAGE_LFBGA176    (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0x1
    #define BSP_PACKAGE_LQFP144     (1)
#else
    #error "ERROR - BSP_CFG_MCU_PART_PACKAGE - Unknown package chosen in r_bsp_config.h"
#endif

/* Memory size of your MCU. */
#if   BSP_CFG_MCU_PART_MEMORY_SIZE == 0x4
    #define BSP_ROM_SIZE_BYTES              (786432)
    #define BSP_RAM_SIZE_BYTES              (131072)
    #define BSP_DATA_FLASH_SIZE_BYTES       (32768)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0x6
    #define BSP_ROM_SIZE_BYTES              (1048576)
    #define BSP_RAM_SIZE_BYTES              (131072)
    #define BSP_DATA_FLASH_SIZE_BYTES       (32768)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0x7
    #define BSP_ROM_SIZE_BYTES              (1572864)
    #define BSP_RAM_SIZE_BYTES              (131072)
    #define BSP_DATA_FLASH_SIZE_BYTES       (32768)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0x8
    #define BSP_ROM_SIZE_BYTES              (2097152)
    #define BSP_RAM_SIZE_BYTES              (131072)
    #define BSP_DATA_FLASH_SIZE_BYTES       (32768)
#else
    #error "ERROR - BSP_CFG_MCU_PART_MEMORY_SIZE - Unknown memory size chosen in r_bsp_config.h"
#endif

/* System clock speed in Hz. */
#define BSP_ICLK_HZ                 (BSP_CFG_XTAL_HZ*BSP_CFG_ICK_MUL)
/* Peripheral clock speed in Hz. */
#define BSP_PCLK_HZ                 (BSP_CFG_XTAL_HZ*BSP_CFG_PCK_MUL)
/* External bus clock speed in Hz. */
#define BSP_BCLK_HZ                 (BSP_CFG_XTAL_HZ*BSP_CFG_BCK_MUL)

/* Null argument definitions. */
#define FIT_NO_FUNC                 ((void (*)(void *))0xA0000000)  //Reserved space on RX
#define FIT_NO_PTR                  (0xA0000000)                    //Reserved space on RX

/* Mininum and maximum IPL levels available for this MCU. */
#define BSP_MCU_IPL_MAX             (0x7)
#define BSP_MCU_IPL_MIN             (0)

#endif /* MCU_INFO */
