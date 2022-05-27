#ifndef __BL_PACKET_H__
#define __BL_PACKET_H__
#include <stdint.h>
#include"tm4c1290nczad.h"
extern int ReceivePacket(uint8_t *pui8Data, uint32_t *pui32Size);
extern int SendPacket(uint8_t *pui8Data, uint32_t ui32Size);
extern void AckPacket(void);

#endif
