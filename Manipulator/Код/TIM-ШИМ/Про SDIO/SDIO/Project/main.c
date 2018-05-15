#include "stm32f10x.h"
#include "sdcard.h"
 
uint32_t writeBuffer[4];
uint32_t readBuffer[4];
SD_CardInfo SDCardInfo;
 
 

int main()
{
    writeBuffer[0] = 0x11111111;
    writeBuffer[1] = 0x22222222;
    writeBuffer[2] = 0x33333333;
    writeBuffer[3] = 0x44444444;

    SD_Init();

    SD_GetCardInfo(&SDCardInfo);

    SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));
    SD_SetDeviceMode(SD_POLLING_MODE);

    SD_WriteBlock(0x00, writeBuffer, 512);
    SD_ReadBlock(0x00, readBuffer, 512);
 
    while(1)
    {
    }
}