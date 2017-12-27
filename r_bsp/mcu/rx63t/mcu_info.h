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
* Device(s)    : RX63T
* Description  : Information about the MCU on this board.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 12.10.2012 1.00     First Release
*         : 19.11.2012 1.10     BSP_ICLK_HZ, BSP_PCLKA_HZ, etc.. now refelect correct values depending on which clock 
*                               was chosen in r_bsp_config.h. Added null argument macros. Updated code to use 'BSP_' and
*                               'BSP_CFG_' prefix for macros.
*         : 18.01.2013 1.20     Added BSP_MCU_IPL_MAX and BSP_MCU_IPL_MIN macros.
*         : 21.01.2013 1.30     Added RX63T-H options.
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

/* This macro means that this MCU is part of the RX63x collection of MCUs (i.e. RX630, RX631/N, RX63T). */
#define BSP_MCU_RX63_ALL            (1)

/* MCU Group name. */
#if   BSP_CFG_MCU_PART_GROUP == 0x0
    #define BSP_MCU_RX630           (1)
#elif BSP_CFG_MCU_PART_GROUP == 0x1
    #define BSP_MCU_RX631           (1)
#elif BSP_CFG_MCU_PART_GROUP == 0x2
    #define BSP_MCU_RX63N           (1)
#elif BSP_CFG_MCU_PART_GROUP == 0x3     
    #define BSP_MCU_RX63T           (1)
#else
    #error "ERROR - BSP_CFG_MCU_PART_GROUP - Unknown MCU Group chosen in r_bsp_config.h"
#endif

/* Package. */
#if   BSP_CFG_MCU_PART_PACKAGE == 0x3
    #define BSP_PACKAGE_LQFP144     (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0x5
    #define BSP_PACKAGE_LQFP100     (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0x8
    #define BSP_PACKAGE_LQFP64      (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0x9
    #define BSP_PACKAGE_LQFP48      (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0xA
    #define BSP_PACKAGE_LQFP120     (1)
#elif BSP_CFG_MCU_PART_PACKAGE == 0xB
    #define BSP_PACKAGE_LQFP112     (1)
#else
    #error "ERROR - BSP_CFG_MCU_PART_PACKAGE - Unknown package chosen in r_bsp_config.h"
#endif

/* Memory size of your MCU. */
#if BSP_CFG_MCU_PART_MEMORY_SIZE == 0x4
    #define BSP_ROM_SIZE_BYTES          (32768)
    #define BSP_RAM_SIZE_BYTES          (8192)
    #define BSP_DATA_FLASH_SIZE_BYTES   (8192)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0x5
    #define BSP_ROM_SIZE_BYTES          (49152)
    #define BSP_RAM_SIZE_BYTES          (8192)
    #define BSP_DATA_FLASH_SIZE_BYTES   (8192)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0x6
    #define BSP_ROM_SIZE_BYTES          (65536)
    #define BSP_RAM_SIZE_BYTES          (8192)
    #define BSP_DATA_FLASH_SIZE_BYTES   (8192)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0xB
    #define BSP_ROM_SIZE_BYTES          (262144)
    #define BSP_RAM_SIZE_BYTES          (24576)
    #define BSP_DATA_FLASH_SIZE_BYTES   (32768)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0xC
    #define BSP_ROM_SIZE_BYTES          (393216)
    #define BSP_RAM_SIZE_BYTES          (32768)
    #define BSP_DATA_FLASH_SIZE_BYTES   (32768)
#elif BSP_CFG_MCU_PART_MEMORY_SIZE == 0xE
    #define BSP_ROM_SIZE_BYTES          (524288)
    #define BSP_RAM_SIZE_BYTES          (49152)
    #define BSP_DATA_FLASH_SIZE_BYTES   (32768)
#else
    #error "ERROR - BSP_CFG_MCU_PART_MEMORY_SIZE - Unknown memory size chosen in r_bsp_config.h"
#endif

/* These macros define clock speeds for fixed speed clocks. */
#define BSP_LOCO_HZ                 (125000)        //Low-speed On-Chip Oscillator

/* Clock source select (CKSEL).
   0 = Low Speed On-Chip Oscillator  (LOCO)
   2 = Main Clock Oscillator  
   4 = PLL Circuit
*/ 
#if   (BSP_CFG_CLOCK_SOURCE == 0)
    #define BSP_SELECTED_CLOCK_HZ   (BSP_LOCO_HZ)       
#elif (BSP_CFG_CLOCK_SOURCE == 2)
    #define BSP_SELECTED_CLOCK_HZ   (BSP_CFG_XTAL_HZ)
#elif (BSP_CFG_CLOCK_SOURCE == 4)
    #define BSP_SELECTED_CLOCK_HZ   ((BSP_CFG_XTAL_HZ/BSP_CFG_PLL_DIV) * BSP_CFG_PLL_MUL)
#endif

/* System clock speed in Hz. */
#define BSP_ICLK_HZ                 (BSP_SELECTED_CLOCK_HZ / BSP_CFG_ICK_DIV)
/* Peripheral Module Clock A speed in Hz. Used for MTU3 and GPT. */
#define BSP_PCLKA_HZ                (BSP_SELECTED_CLOCK_HZ / BSP_CFG_PCKA_DIV)
/* Peripheral Module Clock B speed in Hz. */
#define BSP_PCLKB_HZ                (BSP_SELECTED_CLOCK_HZ / BSP_CFG_PCKB_DIV)
#if defined(BSP_CFG_PCKC_DIV)
/* Peripheral Module Clock C speed in Hz. */
#define BSP_PCLKC_HZ                (BSP_SELECTED_CLOCK_HZ / BSP_CFG_PCKC_DIV)
#endif
/* S12ADclock Clock D speed in Hz. */
#define BSP_PCLKD_HZ                (BSP_SELECTED_CLOCK_HZ / BSP_CFG_PCKD_DIV)
/* FlashIF clock speed in Hz. */
#define BSP_FCLK_HZ                 (BSP_SELECTED_CLOCK_HZ / BSP_CFG_FCK_DIV)
#if defined(BSP_CFG_BCK_DIV)
/* External bus clock speed in Hz. */
#define BSP_BCLK_HZ                 (BSP_SELECTED_CLOCK_HZ / BSP_CFG_PCKD_DIV)
#endif
#if defined(BSP_UCLK_HZ)
/* USB clock speed in Hz. */
#define BSP_UCLK_HZ                 (BSP_SELECTED_CLOCK_HZ / BSP_CFG_UCK_DIV) 
#endif

/* Null argument definitions. */
#define FIT_NO_FUNC                 ((void (*)(void *))0xA0000000)  //Reserved space on RX
#define FIT_NO_PTR                  (0xA0000000)                    //Reserved space on RX
                                                                    
/* Mininum and maximum IPL levels available for this MCU. */
#define BSP_MCU_IPL_MAX             (0xF)
#define BSP_MCU_IPL_MIN             (0)

#endif /* MCU_INFO */

