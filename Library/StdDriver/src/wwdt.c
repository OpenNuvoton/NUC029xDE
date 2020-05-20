/**************************************************************************//**
 * @file     wwdt.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/06/10 10:50a $
 * @brief    NUC029xDE series WWDT driver source file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC029xDE.h"


/** @addtogroup NUC029xDE_Device_Driver NUC029xDE Device Driver
  @{
*/

/** @addtogroup WWDT_Driver WWDT Driver
  @{
*/

/** @addtogroup NUC131_WWDT_EXPORTED_FUNCTIONS WWDT Exported Functions
  @{
*/

/**
  * @brief      Open WWDT and start counting
  *
  * @param[in]  u32PreScale     Pre-scale setting of WWDT counter. Valid values are:
  *                             - \ref WWDT_PRESCALER_1
  *                             - \ref WWDT_PRESCALER_2
  *                             - \ref WWDT_PRESCALER_4
  *                             - \ref WWDT_PRESCALER_8
  *                             - \ref WWDT_PRESCALER_16
  *                             - \ref WWDT_PRESCALER_32
  *                             - \ref WWDT_PRESCALER_64
  *                             - \ref WWDT_PRESCALER_128
  *                             - \ref WWDT_PRESCALER_192
  *                             - \ref WWDT_PRESCALER_256
  *                             - \ref WWDT_PRESCALER_384
  *                             - \ref WWDT_PRESCALER_512
  *                             - \ref WWDT_PRESCALER_768
  *                             - \ref WWDT_PRESCALER_1024
  *                             - \ref WWDT_PRESCALER_1536
  *                             - \ref WWDT_PRESCALER_2048
  * @param[in]  u32CmpValue     Setting the window compared value. Valid values are between 0x0 to 0x3F.
  * @param[in]  u32EnableInt    Enable WWDT time-out interrupt function. Valid values are TRUE and FALSE.
  *
  * @return     None
  *
  * @details    This function makes WWDT module start counting with different counter period by pre-scale setting and compared window value.
  * @note       This WWDTCR register can be write only one time after chip is powered on or reset.
  */
void WWDT_Open(uint32_t u32PreScale,
               uint32_t u32CmpValue,
               uint32_t u32EnableInt)
{
    WWDT->WWDTCR = u32PreScale |
                   (u32CmpValue << WWDT_WWDTCR_WINCMP_Pos) |
                   ((u32EnableInt == TRUE) ? WWDT_WWDTCR_WWDTIE_Msk : 0) |
                   WWDT_WWDTCR_WWDTEN_Msk;
    return;
}

/*@}*/ /* end of group NUC131_WWDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group WWDT_Driver */

/*@}*/ /* end of group NUC029xDE_Device_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
