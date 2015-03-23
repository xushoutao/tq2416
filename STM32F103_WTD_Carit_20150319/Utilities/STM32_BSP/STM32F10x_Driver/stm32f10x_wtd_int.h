#ifndef __STM32F10X_WTD_INT_H__
#define __STM32F10X_WTD_INT_H__

#ifdef __cplusplus
extern "C" {
#endif

void Driver_IntRelocate(void);
void Driver_IntRegister(unsigned long ulInterrupt, void (*pfnHandler)(void));
void Driver_IntUnregister(unsigned long ulInterrupt);

#ifdef __cplusplus
}
#endif


#endif /* __STM32F10X_WTD_INT_H__ */
