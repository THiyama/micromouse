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
* File Name    : vecttbl.c
* Device(s)    : RX610
* Description  : Definition of the fixed vector table and User Boot reset vector.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 26.10.2011 1.00     First Release
*         : 17.02.2012 1.10     Made function names compliant with CS v4.0
*         : 12.03.2012 1.20     ID Code is now specified in r_bsp_config.h. It is still used here in Fixed_Vectors[].
*         : 26.06.2012 1.30     Brought in arrays from flash_options.c into here. Also added optional callbacks to the 
*                               ISRs. Now using sections instead of defining the address for a specific array. Add macro
*                               to set ROM Code Protection.
*         : 16.07.2012 1.40     Added code to handle exception interrupts better.
*         : 19.11.2012 1.50     Updated code to use 'BSP_' and 'BSP_CFG_' prefix for macros.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Fixed size integers. */
#include <stdint.h>
/* Used for nop(). */
#include <machine.h>
/* BSP configuration. */
#include "platform.h"

/***********************************************************************************************************************
* Function name: PowerON_Reset_PC
* Description  : The reset vector points to this function.  Code execution starts in this function after reset.
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
/* BCH - 01/16/2013 */
/* 3447: External linkage is not needed for this special function as it is the function that is run out of reset. */
/* PRQA S 3447 ++ */
extern void PowerON_Reset_PC(void);                      

/***********************************************************************************************************************
* Function name: excep_supervisor_inst_isr
* Description  : Supervisor Instruction Violation ISR
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
#pragma interrupt (excep_supervisor_inst_isr)
void excep_supervisor_inst_isr(void)
{
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_EXCEP_SUPERVISOR_ISR_CALLBACK)
    BSP_CFG_EXCEP_SUPERVISOR_ISR_CALLBACK();

    /* If you do not put the MCU in Supervisor mode before returning then it will just execute the same violating
       instruction again and come back in here. Since the PSW is restored from the stack when returning from the 
       exception, you would need to alter the saved PSW on the stack to change to Supervisor mode. We do not do this 
       here because the only 'safe' way to do this would be to write this function in assembly. Even then most users
       would probably want to handle this someway instead of just going back to the application. */
#else
    nop();
#endif
}

/***********************************************************************************************************************
* Function name: excep_undefined_inst_isr
* Description  : Undefined instruction exception ISR
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
#pragma interrupt (excep_undefined_inst_isr)
void excep_undefined_inst_isr(void)
{
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_EXCEP_UNDEFINED_INSTR_ISR_CALLBACK)
    BSP_CFG_EXCEP_UNDEFINED_INSTR_ISR_CALLBACK();
#else
    nop();
#endif
}

/***********************************************************************************************************************
* Function name: excep_floating_point_isr
* Description  : Floating point exception ISR
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
#pragma interrupt (excep_floating_point_isr)
void excep_floating_point_isr(void)
{
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_EXCEP_FPU_ISR_CALLBACK)
    /* Used for reading FPSW register. */
    uint32_t temp_fpsw;

    BSP_CFG_EXCEP_FPU_ISR_CALLBACK();

    /* Get current FPSW. */
    temp_fpsw = (uint32_t)get_fpsw();
    /* Clear only the FPU exception flags. */
    set_fpsw(temp_fpsw & ((uint32_t)~0x00007C00));
#else
    nop();
#endif
}

/***********************************************************************************************************************
* Function name: non_maskable_isr
* Description  : Non-maskable interrupt ISR
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
#pragma interrupt (non_maskable_isr)
void non_maskable_isr(void)
{
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_NMI_ISR_CALLBACK)
    BSP_CFG_NMI_ISR_CALLBACK();

    /* Clear NMI flag. */
    ICU.NMICLR.BIT.NMICLR = 1;
#else
    nop();
#endif
}

/***********************************************************************************************************************
* Function name: undefined_interrupt_source_isr
* Description  : All undefined interrupt vectors point to this function.
*                Set a breakpoint in this function to determine which source is creating unwanted interrupts.
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
#pragma interrupt (undefined_interrupt_source_isr)
void undefined_interrupt_source_isr(void)
{
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_UNDEFINED_INT_ISR_CALLBACK)
    BSP_CFG_UNDEFINED_INT_ISR_CALLBACK();
#else
    nop();
#endif
}

/***********************************************************************************************************************
* Function name: bus_error_isr
* Description  : By default, this demo code enables the Bus Error Interrupt. This interrupt will fire if the user tries 
*                to access code or data from one of the reserved areas in the memory map, including the areas covered 
*                by disabled chip selects. A nop() statement is included here as a convenient place to set a breakpoint 
*                during debugging and development, and further handling should be added by the user for their 
*                application.
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
#pragma interrupt (bus_error_isr(vect=VECT(BSC,BUSERR)))
void bus_error_isr (void)
{
    /* Clear the bus error */
    BSC.BERCLR.BIT.STSCLR = 1;

    /* 
        To find the address that was accessed when the bus error occurred, read the register BSC.BERSR2.WORD.  The upper
        13 bits of this register contain the upper 13-bits of the offending address (in 512K byte units)
    */
    
    /* If the user defined a callback function in r_bsp_config.h then it will be called here. */
#if defined(BSP_CFG_BUS_ERROR_ISR_CALLBACK)
    BSP_CFG_BUS_ERROR_ISR_CALLBACK();
#else
    nop();
#endif
}

/***********************************************************************************************************************
* The following array fills in the the User Boot reset vector.
***********************************************************************************************************************/
/* When using User Boot Mode the reset vector is fetched from 0xFF7FFFFC instead of 0xFFFFFFFC.
   0xFF7FFFFC - 0xFF7FFFFF : User Boot Reset Vector */

#if BSP_CFG_USER_BOOT_ENABLE == 1

/* Allocate this space in the user boot sector. */
#pragma section C UBSETTINGS 

extern void PowerON_Reset_PC(void);

/* Use this array if you are using User Boot. Make sure to fill in valid address for UB Reset Vector. */
const uint32_t user_boot_settings[1] = 
{
    /* This is the User Boot Reset Vector. When using User Boot put in the reset address here. */
    (uint32_t) PowerON_Reset_PC 
};
#endif

/***********************************************************************************************************************
* The following array fills in the fixed vector table and the code
* protecction ID bytes.
***********************************************************************************************************************/
#pragma section C FIXEDVECT

void* const Fixed_Vectors[] = {
    
    /* 0xffffff80 through 0xffffff9f: Reserved area - must be all 0xFF */
    (void *)0xFFFFFFFF,   /* 0xffffff80 - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff84 - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff88 - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff8C - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff90 - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff94 - Reserved */
    (void *)0xFFFFFFFF,   /* 0xffffff98 - Reserved */

    /* The 32-bit area immediately below (0xffffff9c through 0xffffff9f) is a special area that allows the ROM to be 
       protected from reading or writing by a parallel programmer. Please refer to the HW manual for appropriate 
       settings. The default (all 0xff) places no restrictions and therefore allows reads and writes by a parallel 
       programmer. */
    (void *)BSP_CFG_ROM_CODE_PROTECT_VALUE,   /* 0xffffff9C - ROM Code Protection */

    /* The memory are immediately below (0xffffffa0 through 0xffffffaf) is a special area that allows the on-chip 
       firmware to be protected. See the section "ID Code Protection" in the HW manual for details on how to enable 
       protection. Setting the four long words below to non-0xFF values will enable protection.  Do this only after 
       carefully review the HW manual */
   
    /* 0xffffffA0 through 0xffffffaf: ID Code Protection. The ID code is specified using macros in r_bsp_config.h.  */
    (void *) BSP_CFG_ID_CODE_LONG_1,  /* 0xffffffA0 - Control code and ID code */
    (void *) BSP_CFG_ID_CODE_LONG_2,  /* 0xffffffA4 - ID code (cont.) */
    (void *) BSP_CFG_ID_CODE_LONG_3,  /* 0xffffffA8 - ID code (cont.) */
    (void *) BSP_CFG_ID_CODE_LONG_4,  /* 0xffffffAC - ID code (cont.) */
  
    /* 0xffffffB0 through 0xffffffcf: Reserved area */
    (void *) 0xFFFFFFFF,  /* 0xffffffB0 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffB4 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffB8 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffBC - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffC0 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffC4 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffC8 - Reserved */
    (void *) 0xFFFFFFFF,  /* 0xffffffCC - Reserved */

    /* Fixed vector table */
    /* BCH - 01/16/2013 */
    /* The PRQA tool gives a defect here for casting between pointer-to-function to pointer-to-object since this is 
       undefined behavior in C90. This could be split into a separate array but it would not provide any real benefit.
       For now this is suppressed. */
    /* PRQA S 0307 ++ */
    (void *) excep_supervisor_inst_isr,         /* 0xffffffd0  Exception(Supervisor Instruction) */
    (void *) undefined_interrupt_source_isr,    /* 0xffffffd4  Reserved */
    (void *) undefined_interrupt_source_isr,    /* 0xffffffd8  Reserved */
    (void *) excep_undefined_inst_isr,          /* 0xffffffdc  Exception(Undefined Instruction) */
    (void *) undefined_interrupt_source_isr,    /* 0xffffffe0  Reserved */
    (void *) excep_floating_point_isr,          /* 0xffffffe4  Exception(Floating Point) */
    (void *) undefined_interrupt_source_isr,    /* 0xffffffe8  Reserved */
    (void *) undefined_interrupt_source_isr,    /* 0xffffffec  Reserved */
    (void *) undefined_interrupt_source_isr,    /* 0xfffffff0  Reserved */
    (void *) undefined_interrupt_source_isr,    /* 0xfffffff4  Reserved */
    (void *) non_maskable_isr,                  /* 0xfffffff8  NMI */
    (void *) PowerON_Reset_PC                   /* 0xfffffffc  RESET */
};

