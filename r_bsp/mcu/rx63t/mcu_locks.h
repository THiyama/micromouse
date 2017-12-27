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
* File Name    : mcu_locks.h
* Device(s)    : RX63T
* Description  : This source file has 1 lock per MCU resource.
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 16.11.2012 1.00     First Release
*         : 19.11.2012 1.10     Updated code to use 'BSP_' and 'BSP_CFG_' prefix for macros.
*         : 15.01.2013 1.20     Removed LOCK & _LOCK macros since they were not being used. Added 'undef' for lock type.
*         : 21.01.2013 1.30     Added lock support for previous RX63T-H labeled MCUs.
***********************************************************************************************************************/

#ifndef MCU_LOCKS_H
#define MCU_LOCKS_H

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Gets MCU configuration information. */
#include "r_bsp_config.h"

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
/* This enum defines all of the available hardware locks for this MCU. If you delete an entry out of this list then you
   will decrease the size of the locks array but will not be able to use that lock. For example, if your design is not
   using CAN at all then you can safely remove the BSP_LOCK_CAN# entries below. */
/* Locks for existing RX63T or RX63T-H. */
#if (BSP_CFG_MCU_PART_MEMORY_SIZE <= 0x6)
typedef enum 
{
    BSP_LOCK_BSC = 0,       
    BSP_LOCK_CAC,    
    BSP_LOCK_CMT,    
    BSP_LOCK_CMT0,   
    BSP_LOCK_CMT1,   
    BSP_LOCK_CMT2,   
    BSP_LOCK_CMT3,   
    BSP_LOCK_CRC,    
    BSP_LOCK_DMAC,   
    BSP_LOCK_DMAC0,  
    BSP_LOCK_DMAC1,  
    BSP_LOCK_DMAC2,  
    BSP_LOCK_DMAC3,  
    BSP_LOCK_DOC,    
    BSP_LOCK_DTC,    
    BSP_LOCK_FLASH,  
    BSP_LOCK_GPT,    
    BSP_LOCK_GPT0,   
    BSP_LOCK_GPT1,   
    BSP_LOCK_GPT2,   
    BSP_LOCK_GPT3,   
    BSP_LOCK_ICU,    
    BSP_LOCK_IWDT,   
    BSP_LOCK_MPC,        
    BSP_LOCK_MPU,    
    BSP_LOCK_MTU,    
    BSP_LOCK_MTU0,   
    BSP_LOCK_MTU1,   
    BSP_LOCK_MTU2,   
    BSP_LOCK_MTU3,   
    BSP_LOCK_MTU4,   
    BSP_LOCK_MTU5,   
    BSP_LOCK_MTU6,   
    BSP_LOCK_MTU7,   
    BSP_LOCK_POE,    
    BSP_LOCK_RIIC0,  
    BSP_LOCK_RSPI0,  
    BSP_LOCK_S12AD,  
    BSP_LOCK_SCI0,   
    BSP_LOCK_SCI1,   
    BSP_LOCK_SCI12,  
    BSP_LOCK_SMCI0,  
    BSP_LOCK_SMCI1,  
    BSP_LOCK_SMCI12, 
    BSP_LOCK_SYSTEM, 
    BSP_LOCK_WDT,    
    BSP_NUM_LOCKS //This entry is not a valid lock. It is used for sizing g_bsp_Locks[] array below. Do not touch!
} mcu_lock_t;
#else
/* RX63T-H. */
typedef enum 
{
    BSP_LOCK_AD0 = 0,             
    BSP_LOCK_BSC,      
    BSP_LOCK_CAC,      
    BSP_LOCK_CAN1,     
    BSP_LOCK_CMT,      
    BSP_LOCK_CMT0,     
    BSP_LOCK_CMT1,     
    BSP_LOCK_CMT2,     
    BSP_LOCK_CMT3,     
    BSP_LOCK_CRC,      
    BSP_LOCK_DA,       
    BSP_LOCK_DMAC,     
    BSP_LOCK_DMAC0,    
    BSP_LOCK_DMAC1,    
    BSP_LOCK_DMAC2,    
    BSP_LOCK_DMAC3,    
    BSP_LOCK_DOC,      
    BSP_LOCK_DTC,      
    BSP_LOCK_FLASH,    
    BSP_LOCK_GPT,      
    BSP_LOCK_GPTB,     
    BSP_LOCK_GPT0,     
    BSP_LOCK_GPT1,     
    BSP_LOCK_GPT2,         
    BSP_LOCK_GPT3,     
    BSP_LOCK_GPT4,     
    BSP_LOCK_GPT5,     
    BSP_LOCK_GPT6,     
    BSP_LOCK_GPT7,     
    BSP_LOCK_ICU,      
    BSP_LOCK_IWDT,     
    BSP_LOCK_MPC,      
    BSP_LOCK_MPU,      
    BSP_LOCK_MTU,      
    BSP_LOCK_MTU0,     
    BSP_LOCK_MTU1,     
    BSP_LOCK_MTU2,     
    BSP_LOCK_MTU3,     
    BSP_LOCK_MTU4,     
    BSP_LOCK_MTU5,     
    BSP_LOCK_MTU6,     
    BSP_LOCK_MTU7,     
    BSP_LOCK_POE,      
    BSP_LOCK_PORT,     
    BSP_LOCK_PORT0,    
    BSP_LOCK_PORT1,    
    BSP_LOCK_PORT2,    
    BSP_LOCK_PORT3,    
    BSP_LOCK_PORT4,    
    BSP_LOCK_PORT5,    
    BSP_LOCK_PORT6,    
    BSP_LOCK_PORT7,    
    BSP_LOCK_PORT8,    
    BSP_LOCK_PORT9,    
    BSP_LOCK_PORTA,    
    BSP_LOCK_PORTB,    
    BSP_LOCK_PORTC,    
    BSP_LOCK_PORTD,    
    BSP_LOCK_PORTE,    
    BSP_LOCK_PORTF,    
    BSP_LOCK_PORTG,    
    BSP_LOCK_RIIC0,    
    BSP_LOCK_RIIC1,    
    BSP_LOCK_RSPI0,    
    BSP_LOCK_RSPI1,    
    BSP_LOCK_S12ADB0,  
    BSP_LOCK_S12ADB1,  
    BSP_LOCK_SCI0,     
    BSP_LOCK_SCI1,     
    BSP_LOCK_SCI2,     
    BSP_LOCK_SCI3,     
    BSP_LOCK_SCI12,    
    BSP_LOCK_SMCI0,    
    BSP_LOCK_SMCI1,    
    BSP_LOCK_SMCI2,    
    BSP_LOCK_SMCI3,    
    BSP_LOCK_SMCI12,   
    BSP_LOCK_SYSTEM,   
    BSP_LOCK_USB0,     
    BSP_LOCK_WDT,      
    BSP_NUM_LOCKS //This entry is not a valid lock. It is used for sizing g_bsp_Locks[] array below. Do not touch!
} mcu_lock_t;
#endif

typedef struct
{
    /* The actual lock. int32_t is used because this is what the xchg() instruction takes as parameters. */
    int32_t     lock;
    /* Could add a ID for locking and unlocking. In this could protect against any function being able to unlock. */
} bsp_lock_t;

/***********************************************************************************************************************
Error checking
***********************************************************************************************************************/
#if BSP_CFG_USER_LOCKING_ENABLED == 0
#undef  BSP_CFG_USER_LOCKING_TYPE  
#define BSP_CFG_USER_LOCKING_TYPE       bsp_lock_t
#else
    #if !defined(BSP_CFG_USER_LOCKING_TYPE)
    #error "R_BSP ERROR - If you are using your own locking mechanism then you must define BSP_CFG_USER_LOCKING_TYPE in \
            r_bsp_config.h."
    #endif
#endif

/***********************************************************************************************************************
Exported global variables (to be accessed by other files)
***********************************************************************************************************************/
extern BSP_CFG_USER_LOCKING_TYPE g_bsp_Locks[];    

#endif /* MCU_LOCKS_H */

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
