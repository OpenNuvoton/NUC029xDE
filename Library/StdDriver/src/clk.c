/**************************************************************************//**
 * @file     clk.c
 * @version  V3.00
 * $Revision: 25 $
 * $Date: 15/10/26 8:42a $
 * @brief    NUC029xDE series CLK driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NUC029xDE.h"

/** @addtogroup NUC029xDE_Device_Driver NUC029xDE Device Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup NUC029xDE_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable frequency output function
  * @param      None
  * @return     None
  * @details    This function disable frequency output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->APBCLK &= ~CLK_APBCLK_FDIV_EN_Msk;
}

/**
  * @brief      This function enable frequency divider module clock.
  *             enable frequency divider clock function and configure frequency divider.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL2_FRQDIV_S_HXT
  *             - \ref CLK_CLKSEL2_FRQDIV_S_HCLK
  *             - \ref CLK_CLKSEL2_FRQDIV_S_HIRC
  * @param[in]  u32ClkDiv is divider output frequency selection. It could be 0~15.
  * @param[in]  u32ClkDivBy1En is frequency divided by one enable.
  * @return     None
  *
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv. \n
  *             The formula is: \n
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1) \n
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves. \n
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->FRQDIV = CLK_FRQDIV_DIVIDER_EN_Msk | u32ClkDiv | (u32ClkDivBy1En << CLK_FRQDIV_DIVIDER1_Pos);

    /* Enable CKO clock source */
    CLK->APBCLK |= CLK_APBCLK_FDIV_EN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_FRQDIV_S_Msk)) | (u32ClkSrc);
}

/**
  * @brief      Enter to Power-down mode
  * @param      None
  * @return     None
  * @details    This function is used to let system enter to Power-down mode.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Enter to Idle mode
  * @param      None
  * @return     None
  * @details    This function let system enter to Idle mode.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCON &= ~CLK_PWRCON_PWR_DOWN_EN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Get external high speed crystal clock frequency
  * @param      None
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    if(CLK->PWRCON & CLK_PWRCON_XTL12M_EN_Msk)
        return __HXT;
    else
        return 0;
}


/**
  * @brief      Get HCLK frequency
  * @param      None
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Get PCLK frequency
  * @param      None
  * @return     PCLK frequency
  * @details    This function get PCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Get CPU frequency
  * @param      None
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency. The range of u32Hclk is 25 MHz ~ 50 MHz.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz.
  *             It would configure PLL frequency to 50MHz ~ 100MHz,
  *             set HCLK clock divider as 2 and switch HCLK clock source to PLL.
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;

    /* The range of u32Hclk is 25 MHz ~ 50 MHz */
    if(u32Hclk > FREQ_50MHZ)
        u32Hclk = FREQ_50MHZ;
    if(u32Hclk < FREQ_25MHZ)
        u32Hclk = FREQ_25MHZ;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= (~CLK_CLKDIV_HCLK_N_Msk);

    /* Configure PLL setting if HXT clock is stable */
    if(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk)
        u32Hclk = CLK_EnablePLL(CLK_PLLCON_PLL_SRC_HXT, (u32Hclk << 1));

    /* Configure PLL setting if HXT clock is not stable */
    else
    {
        u32Hclk = CLK_EnablePLL(CLK_PLLCON_PLL_SRC_HIRC, (u32Hclk << 1));

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;
    }

    /* Select HCLK clock source to PLL,
       Select HCLK clock source divider as 2
       and update system core clock
    */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if(u32HIRCSTB == 0)
        CLK->PWRCON &= ~CLK_PWRCON_OSC22M_EN_Msk;

    /* Return actually HCLK frequency is PLL frequency divide 2 */
    return u32Hclk >> 1;
}

/**
  * @brief      Set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLK_S_HXT
  *             - \ref CLK_CLKSEL0_HCLK_S_PLL
  *             - \ref CLK_CLKSEL0_HCLK_S_LIRC
  *             - \ref CLK_CLKSEL0_HCLK_S_HIRC
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV_HCLK(x)
  * @return     None
  * @details    This function set HCLK clock source and HCLK clock divider.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk;

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;

    /* Apply new Divider */
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | u32ClkDiv;

    /* Switch to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if(u32HIRCSTB == 0)
        CLK->PWRCON &= ~CLK_PWRCON_OSC22M_EN_Msk;
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                          |Divider                 |
  * | :----------------  | :----------------------------------  | :--------------------- |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDT_S_LIRC           | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDT_S_HCLK_DIV2048   | x                      |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_HXT            |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_PLL            |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_HCLK           |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_HIRC           |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL1_SPI0_S_HCLK          | x                      |
  * |\ref SPI0_MODULE    |\ref CLK_CLKSEL1_SPI0_S_PLL           | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_HXT           | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_HCLK          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_EXT_TRG       | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_LIRC          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_HIRC          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_HXT           | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_HCLK          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_EXT_TRG       | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_LIRC          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_HIRC          | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2_S_HXT           | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2_S_HCLK          | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2_S_EXT_TRG       | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2_S_LIRC          | x                      |
  * |\ref TMR2_MODULE    |\ref CLK_CLKSEL1_TMR2_S_HIRC          | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3_S_HXT           | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3_S_HCLK          | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3_S_EXT_TRG       | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3_S_LIRC          | x                      |
  * |\ref TMR3_MODULE    |\ref CLK_CLKSEL1_TMR3_S_HIRC          | x                      |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART_S_HXT           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART_S_PLL           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART0_MODULE   |\ref CLK_CLKSEL1_UART_S_HIRC          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UART_S_HXT           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UART_S_PLL           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART1_MODULE   |\ref CLK_CLKSEL1_UART_S_HIRC          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART2_MODULE   |\ref CLK_CLKSEL1_UART_S_HXT           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART2_MODULE   |\ref CLK_CLKSEL1_UART_S_PLL           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART2_MODULE   |\ref CLK_CLKSEL1_UART_S_HIRC          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART3_MODULE   |\ref CLK_CLKSEL1_UART_S_HXT           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART3_MODULE   |\ref CLK_CLKSEL1_UART_S_PLL           |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART3_MODULE   |\ref CLK_CLKSEL1_UART_S_HIRC          |\ref CLK_CLKDIV_UART(x) |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_S_HXT         | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_S_HCLK        | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_S_HIRC        | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDT_S_HCLK_DIV2048  | x                      |
  * |\ref WWDT_MODULE    |\ref CLK_CLKSEL2_WWDT_S_LIRC          | x                      |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL3_PWM0_S_PLL           | x                      |
  * |\ref PWM0_MODULE    |\ref CLK_CLKSEL3_PWM0_S_PCLK          | x                      |
  * |\ref PWM1_MODULE    |\ref CLK_CLKSEL3_PWM1_S_PLL           | x                      |
  * |\ref PWM1_MODULE    |\ref CLK_CLKSEL3_PWM1_S_PCLK          | x                      |
  */

void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32sel = 0, u32div = 0;
    uint32_t au32SelTbl[4] = {0x0, 0x4, 0xC, 0x24};

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32div = (uint32_t)&CLK->CLKDIV + ((MODULE_CLKDIV(u32ModuleIdx)) * 4);
        /* Apply new divider */
        M32(u32div) = (M32(u32div) & (~(MODULE_CLKDIV_Msk(u32ModuleIdx) << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + (au32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);
        /* Set new clock selection setting */
        M32(u32sel) = (M32(u32sel) & (~(MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
    }

}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLK_S_HXT
  *             - \ref CLK_CLKSEL0_STCLK_S_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLK_S_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLK_S_HIRC_DIV2
  * @return     None
  * @details    This function set SysTick clock source.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLK_S_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCON_XTL12M_EN_Msk
  *             - \ref CLK_PWRCON_OSC22M_EN_Msk
  *             - \ref CLK_PWRCON_OSC10K_EN_Msk
  * @return     None
  * @details    This function enable clock source.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCON |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCON_XTL12M_EN_Msk
  *             - \ref CLK_PWRCON_OSC22M_EN_Msk
  *             - \ref CLK_PWRCON_OSC10K_EN_Msk
  * @return     None
  * @details    This function disable clock source.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCON &= ~u32ClkMask;
}

/**
  * @brief      Enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref ISP_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref FDIV_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref PWM1_MODULE
  *             - \ref ADC_MODULE
  * @return     None
  * @details    This function enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t au32OffsetTbl[4] = {0x0, 0x4, 0x2C, 0x0};

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + au32OffsetTbl[MODULE_APBCLK(u32ModuleIdx)])  |= 1 << MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief      Disable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref ISP_MODULE
  *             - \ref WDT_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref FDIV_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref PWM1_MODULE
  *             - \ref ADC_MODULE
  * @return     None
  * @details    This function disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t au32OffsetTbl[4] = {0x0, 0x4, 0x2C, 0x0};

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK + au32OffsetTbl[MODULE_APBCLK(u32ModuleIdx)])  &= ~(1 << MODULE_IP_EN_Pos(u32ModuleIdx));
}


/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCON_PLL_SRC_HXT
  *             - \ref CLK_PLLCON_PLL_SRC_HIRC
  * @param[in]  u32PllFreq is PLL frequency. The range of u32PllFreq is 25 MHz ~ 200 MHz.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCON register to set specified PLL frequency.
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32CLK_SRC;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Min, u32MinNF, u32MinNR;

    /* Disable PLL first to avoid unstable when setting PLL. */
    CLK->PLLCON = CLK_PLLCON_PD_Msk;

    /* PLL source clock is from HXT */
    if(u32PllClkSrc == CLK_PLLCON_PLL_SRC_HXT)
    {
        /* Enable HXT clock */
        CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

        /* Select PLL source clock from HXT */
        u32CLK_SRC = CLK_PLLCON_PLL_SRC_HXT;
        u32PllSrcClk = __HXT;

        /* u32NR start from 2 */
        u32NR = 2;
    }

    /* PLL source clock is from HIRC */
    else
    {
        /* Enable HIRC clock */
        CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

        /* Wait for HIRC clock ready */
        CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

        /* Select PLL source clock from HIRC */
        u32CLK_SRC = CLK_PLLCON_PLL_SRC_HIRC;
        u32PllSrcClk = __HIRC;

        /* u32NR start from 4 when FIN = 22.1184MHz to avoid calculation overflow */
        u32NR = 4;
    }

    /* Select "NO" according to request frequency */
    if((u32PllFreq <= FREQ_200MHZ) && (u32PllFreq > FREQ_100MHZ))
    {
        u32NO = 0;
    }
    else if((u32PllFreq <= FREQ_100MHZ) && (u32PllFreq > FREQ_50MHZ))
    {
        u32NO = 1;
        u32PllFreq = u32PllFreq << 1;
    }
    else if((u32PllFreq <= FREQ_50MHZ) && (u32PllFreq >= FREQ_25MHZ))
    {
        u32NO = 3;
        u32PllFreq = u32PllFreq << 2;
    }
    else
    {
        /* Wrong frequency request. Just return default setting. */
        goto lexit;
    }

    /* Find best solution */
    u32Min = (uint32_t) - 1;
    u32MinNR = 0;
    u32MinNF = 0;
    for(; u32NR <= 33; u32NR++)
    {
        u32Tmp = u32PllSrcClk / u32NR;
        if((u32Tmp > 1600000) && (u32Tmp < 15000000))
        {
            for(u32NF = 2; u32NF <= 513; u32NF++)
            {
                u32Tmp2 = u32Tmp * u32NF;
                if((u32Tmp2 >= 100000000) && (u32Tmp2 <= 200000000))
                {
                    u32Tmp3 = (u32Tmp2 > u32PllFreq) ? u32Tmp2 - u32PllFreq : u32PllFreq - u32Tmp2;
                    if(u32Tmp3 < u32Min)
                    {
                        u32Min = u32Tmp3;
                        u32MinNR = u32NR;
                        u32MinNF = u32NF;

                        /* Break when get good results */
                        if(u32Min == 0)
                            break;
                    }
                }
            }
        }
    }

    /* Enable and apply new PLL setting. */
    CLK->PLLCON = u32CLK_SRC | (u32NO << 14) | ((u32MinNR - 2) << 9) | (u32MinNF - 2);

    /* Waiting for PLL clock stable */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);

    /* Return actual PLL output clock frequency */
    return u32PllSrcClk / ((u32NO + 1) * u32MinNR) * u32MinNF;

lexit:

    /* Apply default PLL setting and return */
    if(u32PllClkSrc == CLK_PLLCON_PLL_SRC_HXT)
        CLK->PLLCON = 0xC22E; /* 48MHz */
    else
        CLK->PLLCON = 0x8D66F; /* 48.06498462MHz */

    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);
    return CLK_GetPLLClockFreq();


}

/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function disable PLL.
  */
void CLK_DisablePLL(void)
{
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_CLKSTATUS_XTL12M_STB_Msk
  *             - \ref CLK_CLKSTATUS_OSC22M_STB_Msk
  *             - \ref CLK_CLKSTATUS_OSC10K_STB_Msk
  *             - \ref CLK_CLKSTATUS_PLL_STB_Msk
  *
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  *
  * @details    To wait for clock ready by specified CLKSTATUS bit or timeout (>500ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = SystemCoreClock>>1; /* 500ms time-out */

    while((CLK->CLKSTATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }

    return 1;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STCLK_S_HXT
  *             - \ref CLK_CLKSEL0_STCLK_S_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STCLK_S_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STCLK_S_HIRC_DIV2
  *             - \ref CLK_CLKSEL0_STCLK_S_HCLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt.
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;

    /* Set System Tick clock source */
    if( u32ClkSrc == CLK_CLKSEL0_STCLK_S_HCLK )
        /* Select System Tick clock source from core clock */
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    else
    {
        /* Select System Tick external reference clock source */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLK_S_Msk) | u32ClkSrc;

        /* Select System Tick clock source from external reference clock */
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
    }

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
	SysTick->CTRL = 0;
}


/*@}*/ /* end of group NUC029xDE_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CLK_Driver */

/*@}*/ /* end of group NUC029xDE_Device_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
