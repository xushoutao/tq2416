#ifndef __STM32F10X_WTD_EEPROM_H__
#define __STM32F10X_WTD_EEPROM_H__


#define E2P_MBID	(0x10)
#define E2P_MBID_BK	(0x11)
#define	E2P_WDH		(0x12)
#define E2P_WDL		(0x13)
#define E2P_WSH		(0x14)
#define E2P_WSL		(0x15)
#define E2P_K		(0x16)
#define E2P_K_BK	(0x17)

#define E2P_ALARMTEL_STARTADDR (0x30)
#define E2P_ALARMTEL_BK_STARTADDR (0x40)

typedef enum{
	E2P_OK=0,
	E2P_NG,
	E2P_DEF,
}E2PRETType;

void E2P_Init(void);
E2PRETType E2P_WriteBKUP(const uint8_t address, const uint8_t dat);
E2PRETType E2P_ReadBKUP(const uint8_t address, uint8_t *dat);
E2PRETType E2P_Read16BKUP(const uint8_t address, uint16_t *dat);
E2PRETType E2P_Write16BKUP(const uint8_t address, const uint16_t dat);

#endif /* __STM32F10X_WTD_EEPROM_H__ */
