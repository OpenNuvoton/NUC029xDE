/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/06 10:01a $
 * @brief    FMC LDROM IAP sample program for NUC029xDE series MCU
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC029xDE.h"

#ifdef __GNUC__
#define printf(...)
#endif

#define PLL_CLOCK           50000000

void ProcessHardFault(void){}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}




int main()
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected register */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n\n");
    printf("NUC029xDE FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\nTo branch to APROM...\n");

    printf("\n\nChange VECMAP and branch to APROM...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
    __set_PRIMASK(1);

    /* Change VECMAP for booting to APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);

    /* Lock protected Register */
    SYS_LockReg();

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
