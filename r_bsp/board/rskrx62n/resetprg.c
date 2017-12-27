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
* File Name    : resetprg.c
* Device(s)    : RX62x
* Description  : Defines post-reset routines that are used to configure the MCU prior to the main program starting. 
*                This is were the program counter starts on power-up or reset.
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 26.10.2011 1.00     First Release
*         : 12.03.2012 1.10     Stack sizes are now defined in r_bsp_config.h. Because of this the #include for 
*                               stacksct.h was removed. Settings for SCKCR are now set in r_bsp_config.h and used here
*                               to setup clocks based on user settings.
*         : 16.07.2012 1.20     Added ability to enable FPU Exceptions based on the BSP_CFG_EXCEP_FPU_ISR_CALLBACK macro
*                               and the ability to enable NMI interrupts based on the BSP_CFG_NMI_ISR_CALLBACK. Also 
*                               added code to enable BCLK and SDCLK output based on settings in r_bsp_config.h.
*         : 09.08.2012 1.30     Added checking of BSP_CFG_IO_LIB_ENABLE macro for calling I/O Lib functions.
*         : 19.11.2012 1.40     Updated code to use 'BSP_' and 'BSP_CFG_' prefix for macros.
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
/* Defines machine level functions used in this file */
#include    <machine.h>
/* Defines MCU configuration functions used in this file */
#include    <_h_c_lib.h>
/* Defines standard variable types used in this file */
#include    <stdbool.h>
#include    <stdint.h>

/* This macro is here so that the stack will be declared here. This is used to prevent multiplication of stack size. */
#define     BSP_DECLARE_STACK
/* Define the target platform */
#include    "platform.h"

/* BCH - 01/16/2013 */
/* 0602: Defect for macro names with '_[A-Z]' is also being suppressed since these are defaut names from toolchain.  
   3447: External linkage is not needed for these toolchain supplied library functions. */
/* PRQA S 0602, 3447 ++ */

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define PSW_init  (0x00030000)
#define FPSW_init (0x00000100)

/***********************************************************************************************************************
Pre-processor Directives
***********************************************************************************************************************/
/* Declare the contents of the function 'Change_PSW_PM_to_UserMode' as
   assembler to the compiler */
#pragma inline_asm Change_PSW_PM_to_UserMode

/* Set this as the entry point from a power-on reset */
#pragma entry PowerON_Reset_PC

/***********************************************************************************************************************
External function Prototypes
***********************************************************************************************************************/
/* Functions to setup I/O library */
extern void _INIT_IOLIB(void);
extern void _CLOSEALL(void);

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/
/* Power-on reset function declaration */
void PowerON_Reset_PC(void);

#if BSP_CFG_RUN_IN_USER_MODE==1
    #if __RENESAS_VERSION__ < 0x01010000
    /* MCU usermode switcher function declaration */
    static void Change_PSW_PM_to_UserMode(void);
    #endif
#endif

/* Main program function delcaration */
void main(void);
static void operating_frequency_set(void);

/***********************************************************************************************************************
* Function name: PowerON_Reset_PC
* Description  : This function is the MCU's entry point from a power-on reset.
*                The following steps are taken in the startup code:
*                1. The User Stack Pointer (USP) and Interrupt Stack Pointer (ISP) are both set immediately after entry 
*                   to this function. The USP and ISP stack sizes are set in the file stacksct.h.
*                   Default sizes are USP=4K and ISP=1K.
*                2. The interrupt vector base register is set to point to the beginning of the relocatable interrupt 
*                   vector table.
*                3. The MCU is setup for floating point operations by setting the initial value of the Floating Point 
*                   Status Word (FPSW).
*                4. The MCU operating frequency is set by configuring the Clock Generation Circuit (CGC) in
*                   operating_frequency_set.
*                5. Calls are made to functions to setup the C runtime environment which involves initializing all 
*                   initialed data, zeroing all uninitialized variables, and configuring STDIO if used
*                   (calls to _INITSCT and _INIT_IOLIB).
*                6. Board-specific hardware setup, including configuring I/O pins on the MCU, in hardware_setup.
*                7. Global interrupts are enabled by setting the I bit in the Program Status Word (PSW), and the stack 
*                   is switched from the ISP to the USP.  The initial Interrupt Priority Level is set to zero, enabling 
*                   any interrupts with a priority greater than zero to be serviced.
*                8. The processor is optionally switched to user mode.  To run in user mode, set the macro 
*                   BSP_CFG_RUN_IN_USER_MODE above to a 1.
*                9. The bus error interrupt is enabled to catch any accesses to invalid or reserved areas of memory.
*
*                Once this initialization is complete, the user's main() function is called.  It should not return.
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
void PowerON_Reset_PC(void)
{
    /* Stack pointers are setup prior to calling this function - see comments above */    
    
    /* Initialise the MCU processor word */
#if __RENESAS_VERSION__ >= 0x01010000    
    set_intb((void *)__sectop("C$VECT"));
#else
    set_intb((unsigned long)__sectop("C$VECT"));
#endif    

    /* Initialize FPSW for floating-point operations */
#ifdef __ROZ
#define FPU_ROUND 0x00000001  /* Let FPSW RMbits=01 (round to zero) */
#else 
#define FPU_ROUND 0x00000000  /* Let FPSW RMbits=00 (round to nearest) */
#endif 
#ifdef __DOFF 
#define FPU_DENOM 0x00000100  /* Let FPSW DNbit=1 (denormal as zero) */
#else 
#define FPU_DENOM 0x00000000  /* Let FPSW DNbit=0 (denormal as is) */
#endif 
#ifdef BSP_CFG_EXCEP_FPU_ISR_CALLBACK 
#define FPU_EXCEP 0x00007C00  /* Let FPSW EV, EO, EZ, EU, EX=1 (FPU exceptions enabled.) */
#else 
#define FPU_EXCEP 0x00000000  /* Let FPSW EV, EO, EZ, EU, EX=1 (FPU exceptions disabled.) */
#endif 
    set_fpsw(FPSW_init | FPU_ROUND | FPU_DENOM | FPU_EXCEP); 
    
#ifdef BSP_CFG_NMI_ISR_CALLBACK
    /* Enable NMI interrupt if callback is configured in r_bsp_config.h */
    ICU.NMIER.BIT.NMIEN = 1;
#endif

    /* Switch to high-speed operation */
    operating_frequency_set();

    /* Initialize C runtime environment */
    _INITSCT();

#if BSP_CFG_IO_LIB_ENABLE == 1
    /* Comment this out if not using I/O lib */
    _INIT_IOLIB();
#endif

    /* Configure the MCU and board hardware */
    hardware_setup();

    /* Change the MCU's usermode from supervisor to user */        
    nop();
    set_psw(PSW_init);      
#if BSP_CFG_RUN_IN_USER_MODE==1
    /* Use chg_pmusr() intrinsic if possible. */
    #if __RENESAS_VERSION__ >= 0x01010000
    chg_pmusr() ;
    #else
    Change_PSW_PM_to_UserMode();
    #endif
#endif

    /* Enable the bus error interrupt to catch accesses to illegal/reserved areas of memory */
    /* The ISR for this interrupt can be found in vecttbl.c in the function "bus_error_isr" */
    /* Clear any pending interrupts */
    IR(BSC,BUSERR) = 0;
    /* Make this the highest priority interrupt (adjust as necessary for your application */
    IPR(BSC,BUSERR) = 0x0F; 
    /* Enable the interrupt in the ICU*/
    IEN(BSC,BUSERR) = 1; 
    /* Enable illegal address interrupt in the BSC */
    BSC.BEREN.BIT.IGAEN = 1;

    /* Call the main program function (should not return) */
    main();
    
#if BSP_CFG_IO_LIB_ENABLE == 1
    /* Comment this out if not using I/O lib - cleans up open files */
    _CLOSEALL();
#endif

    /* BCH - 01/16/2013 */
    /* Infinite loop is intended here. */    
    while(1) /* PRQA S 2740 */
    {
        /* Infinite loop. Put a breakpoint here if you want to catch an exit of main(). */
    }
}

/***********************************************************************************************************************
* Function name: operating_frequency_set
* Description  : Configures the clock settings for each of the device clocks
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
static void operating_frequency_set (void)
{
    /* Used for constructing value to write to SCKCR register. */
    uint32_t temp_clock = 0;

    /* 
    DEFAULT SETTINGS:
    Clock Description              Frequency
    ----------------------------------------
    Input Clock Frequency..............12MHz
    Internal Clock Frequency...........96MHz
    Peripheral Clock Frequency.........48MHz
    External Bus Clock Frequency.......12MHz */
    
    /* SCKCR - System Clock Control Register 
    b27:b23 ICK[3:0] = 0, system clock = EXTAL * 8 = 96 MHz
    b23     PSTOP1 = 1, BCLK pin output disabled
    b22     PSTOP0 = 1, SDCLK pin output disabled
    b19:b16 BCK[3:0] = 3, BCLK set to lowest multiplier (x1)to reduce power
    b11:b8  PCK[3:0] = 1, PCK = EXTAL * 4 = 48 MHz
    */

    /* Figure out setting for ICK bits. */
#if   BSP_CFG_ICK_MUL == 8
    /* Do nothing since ICK bits should be 0. */
#elif BSP_CFG_ICK_MUL == 4
    temp_clock |= 0x01000000;
#elif BSP_CFG_ICK_MUL == 2
    temp_clock |= 0x02000000;
#elif BSP_CFG_ICK_MUL == 1
    temp_clock |= 0x03000000;
#else
    #error "Error! Invalid setting for BSP_CFG_ICK_MUL in r_bsp_config.h"
#endif

    /* Figure out setting for BCK bits. */
#if   BSP_CFG_BCK_MUL == 8
    /* Do nothing since BCK bits should be 0. */
#elif BSP_CFG_BCK_MUL == 4
    temp_clock |= 0x00010000;
#elif BSP_CFG_BCK_MUL == 2
    temp_clock |= 0x00020000;
#elif BSP_CFG_BCK_MUL == 1
    temp_clock |= 0x00030000;
#else
    #error "Error! Invalid setting for BSP_CFG_BCK_MUL in r_bsp_config.h"
#endif

    /* Figure out setting for BCK bits. */
#if   BSP_CFG_PCK_MUL == 8
    /* Do nothing since BCK bits should be 0. */
#elif BSP_CFG_PCK_MUL == 4
    temp_clock |= 0x00000100;
#elif BSP_CFG_PCK_MUL == 2
    temp_clock |= 0x00000200;
#elif BSP_CFG_PCK_MUL == 1
    temp_clock |= 0x00000300;
#else
    #error "Error! Invalid setting for BSP_CFG_PCK_MUL in r_bsp_config.h"
#endif

    /* Configure PSTOP1 bit for BCLK output. */
#if BSP_CFG_BCLK_OUTPUT == 0    
    /* Set PSTOP1 bit */
    temp_clock |= 0x00800000;
#elif BSP_CFG_BCLK_OUTPUT == 1
    /* Clear PSTOP1 bit */
    temp_clock &= ~0x00800000;
#elif BSP_CFG_BCLK_OUTPUT == 2
    /* Clear PSTOP1 bit */
    temp_clock &= ~0x00800000;
    /* Set BCLK divider bit */
    SYSTEM.BCKCR.BIT.BCLKDIV = 1;
#else
    #error "Error! Invalid setting for BSP_CFG_BCLK_OUTPUT in r_bsp_config.h"
#endif

    /* Configure PSTOP0 bit for SDCLK output. */
#if BSP_CFG_SDCLK_OUTPUT == 0    
    /* Set PSTOP0 bit */
    temp_clock |= 0x00400000;
#elif BSP_CFG_SDCLK_OUTPUT == 1
    /* Clear PSTOP0 bit */
    temp_clock &= ~0x00400000;
#else
    #error "Error! Invalid setting for BSP_CFG_SDCLK_OUTPUT in r_bsp_config.h"
#endif

    SYSTEM.SCKCR.LONG = temp_clock;
}

/***********************************************************************************************************************
* Function name: Change_PSW_PM_to_UserMode
* Description  : Assembler function, used to change the MCU's usermode from supervisor to user.
* Arguments    : none
* Return value : none
***********************************************************************************************************************/
#if BSP_CFG_RUN_IN_USER_MODE==1
    #if __RENESAS_VERSION__ < 0x01010000
static void Change_PSW_PM_to_UserMode(void)
{
    MVFC   PSW,R1
    OR     #00100000h,R1
    PUSH.L R1
    MVFC   PC,R1
    ADD    #10,R1
    PUSH.L R1
    RTE
    NOP
    NOP
}
    #endif
#endif
