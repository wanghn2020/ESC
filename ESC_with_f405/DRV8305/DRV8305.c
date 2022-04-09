
#include "main.h"
#include "usbd_cdc_if.h"
#include "spi.h"
#include "gpio.h"


uint8_t spiRxBuf[2], spiTxBuf[2];
uint8_t usbTxBuf[20] = "Hello World!!!\r\n";

uint16_t FAULT_N;

void DRV8305_Init(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);//ENABLE GATE

	while (FAULT_N == 0)
	{
		FAULT_N = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
	}

	spiTxBuf[0] = 0x16;//bit8:7 = 10, PWM with one input.
	spiTxBuf[1] = 0x3b;//write HS Gate Drive Control (Address = 0x7)
	HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);

}


void Clear_DRV8305_Fault(void)
{
	FAULT_N = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
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
        spiTxBuf[1] = 0xa0;//读HS Gate Drive Control (Address = 0x4)
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
        //printf("addr_04=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
        //CDC_Transmit_FS(spiRxBuf, 2);

        HAL_Delay(5000);
        spiTxBuf[0] |= 0x02;
        spiTxBuf[1] |= 0x48;//写IC Operation (Address = 0x9), clear error flag
        HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
    }

}


void Phase_Align(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14, GPIO_PIN_SET);//INLA = 1, INLB = 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//INHB = 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//INHC = 0
}

void Phase_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_14, GPIO_PIN_RESET);//INLA = 0, INLB = 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);//INHB = 0, INHC = 0

}


void Read_DRV8305_REG(void)
{
	spiTxBuf[0] = 0x00;
	spiTxBuf[1] = 0xb8;//"0xb8" is read Gate Drive Control (Address = 0x7)
	HAL_SPI_TransmitReceive(&hspi1, spiTxBuf, spiRxBuf, 2, 50);
	usb_printf("HS_Gate_Drive_Control_REG=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
	//switch (Addr)
	//{
	//	case (0xb8):
	//		usb_printf("HS_Gate_Drive_Control_REG=%x_%x\r\n",spiRxBuf[1], spiRxBuf[0]);
	//		break;
	//}

}
