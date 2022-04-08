
#include "main.h"
#include "usbd_cdc_if.h"
#include "spi.h"

uint8_t spiRxBuf[2], spiTxBuf[2];
uint8_t usbTxBuf[20] = "Hello World!!!\r\n";

void ClearFault(void)
{
	uint16_t FAULT_N = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);

    if (FAULT_N == 0)
    {
        spiTxBuf[0] = 0x00;
        spiTxBuf[1] = 0x88;//读HS Gate Drive Control (Address = 0x1)
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
       // printf("addr_01=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
        usb_printf("addr_01=%x_%x\r\n",spiRxBuf, 2);
        //HAL_Delay(1000);
        spiTxBuf[1] = 0x90;//读HS Gate Drive Control (Address = 0x2)
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
        //printf("addr_02=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
        //CDC_Transmit_FS(spiRxBuf, 2);
        //HAL_Delay(1000);
        spiTxBuf[1] = 0x98;//读HS Gate Drive Control (Address = 0x3)
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
        //printf("addr_03=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
        //CDC_Transmit_FS(spiRxBuf, 2);
        //HAL_Delay(1000);
        spiTxBuf[1] = 0xa0;//读HS Gate Drive Control (Address = 0x1)
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
        //printf("addr_04=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
        //CDC_Transmit_FS(spiRxBuf, 2);

        HAL_Delay(5000);
        spiTxBuf[0] |= 0x02;
        spiTxBuf[1] |= 0x48;//写IC Operation (Address = 0x9), clear error flag
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
    }

}
