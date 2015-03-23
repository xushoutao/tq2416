/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WTD_CONF_H__
#define __WTD_CONF_H__

#include "wtd_e2paddr.h"

#define WTD440X 0 /* 16DI */
#define WTD450C	1 /* 16DO */
#define WTD466C 0 /* 6路继电器输出 */
#define WTD478C 0 /* 8路DI，8路DO */

#if ((WTD440X) || (WTD450C) || (WTD478C))
  #if (WTD440X)
    #define DEV_MODULE_NAME "440X"
  #elif (WTD450C)
    #define DEV_MODULE_NAME "450C"
  #elif (WTD478C)
    #define DEV_MODULE_NAME "478C"
  #endif /* (WTD440X) */
  
  #define WTD_DIGITAL_EN 1
  #if WTD_DIGITAL_EN
    #include "stm32f10x_wtd_digital_if.h"
  #endif /* #if WTD_DIGITAL_EN */

#elif (WTD466C)
  #define DEV_MODULE_NAME "466C"

  #define WTD_RELAY_EN 1
  #if WTD_RELAY_EN
    #include "stm32f10x_wtd_relay_if.h"
  #endif /* #if WTD_RELAY_EN */
#endif /* #if ((WTD440X) || (WTD450C) || (WTD478C)) */



#endif /* __WTD_CONF_H__ */
