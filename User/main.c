/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "usart.h"
#include "LCD/LCD.h"
#include "camera/dcmi_OV7670.h"

/** @addtogroup STM32F2xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup IOToggle
  * @{
  */ 
void LCD_CS_ResetBits(void);
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
static __IO uint32_t TimingDelay;

uint32_t	fps=0;
uint32_t	disfps=0;
uint32_t	fpsbuff[5];
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{	
	OV7670_IDTypeDef OV7670ID;
	RCC_ClocksTypeDef SYS_Clocks;
	uint8_t i=0, j=0;
	uint8_t lcd_buff1[10];	
	
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f2xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f2xx.c file
  */
	if (SysTick_Config(SystemCoreClock / 1000))
  	{ 
	    /* Capture error */ 
	    while (1);
  	}
	RCC_GetClocksFreq(&SYS_Clocks);

	USART_Configuration();
	//USART_NVIC_Config();

	printf("\r\nSYSCLK:%dM\r\n",SYS_Clocks.SYSCLK_Frequency/1000000);
	printf("HCLK:%dM\r\n",SYS_Clocks.HCLK_Frequency/1000000);
	printf("PCLK1:%dM\r\n",SYS_Clocks.PCLK1_Frequency/1000000);
	printf("PCLK2:%dM\r\n",SYS_Clocks.PCLK2_Frequency/1000000);

	LCD_Initializtion();
  	LCD_Clear(Red);

	//GUI_Text(100,104,"Camera Init..",White,Blue);

	printf("\r\n\r\nWelcome to WaveShare STM32F2 series MCU Board Open207Z\r\n");
	printf("OV7670 Init..\r\n");
	if(DCMI_OV7670_Init()==0){
		printf("Camera Have Init..\r\n");
		//GUI_Text(100,120,"Camera Have Init..",White,Blue);
		}
	else {
		printf("OV7670 Init fails!!\r\n");
		//GUI_Text(100,120,"OV7670 Init fails!!",White,Blue);
		}

		DCMI_OV7670_ReadID(&OV7670ID);
	if(DCMI_OV7670_ReadID(&OV7670ID)==0)
	{	
		if(OV7670ID.Manufacturer_ID1==0x7f || OV7670ID.Manufacturer_ID2==0xa2 ||OV7670ID.Version==0x73 ||OV7670ID.PID==0x76)
			printf("OV7670 ID:0x%x 0x%x 0x%x 0x%x\r\n",OV7670ID.Manufacturer_ID1, OV7670ID.Manufacturer_ID2, OV7670ID.PID, OV7670ID.Version);
		else
			printf("OV7670 ID is Error!\r\n");
	}
	/*Set LCD direction*/		
	LCD_WriteReg(0x0011,0x6068);
	LCD_SetCursor(0,0);
	Prepare_Write_RAM();
//	LCD_DrawLine(0,0,319,0,White);
//	LCD_DrawLine(0,0,0,239,White);																										  
//	LCD_DrawLine(0,239,319,239,White);	
//	LCD_DrawLine(319,0,319,239,White);

//		for (i=0; i<241; i++)
//	{
//		 if (i > 100)
//			*(uint16_t *) (FSMC_LCD_ADDRESS)	= Yellow;
//		 else
//			*(uint16_t *) (FSMC_LCD_ADDRESS)	= Green;
//	}
//	Delay(100);
//		for (i=0; i<240; i++)
//	{
//		//LCD_SetCursor(i,i);
//		 if (i > 150)
//			LCD_WriteData(Yellow);
//		 else
//			LCD_WriteData(Green);
//	}

//	*(uint16_t *) (FSMC_LCD_ADDRESS + 40)	= White;
	
	/* Start Image capture and Display on the LCD *****************************/
    /* Enable DMA transfer */
    DMA_Cmd(DMA2_Stream1, ENABLE);

    /* Enable DCMI interface */
    DCMI_Cmd(ENABLE); 

    /* Start Image capture */ 
		DCMI_CaptureCmd(ENABLE);	
	
	//DCMI_SingleRandomWrite(0x70, 0x80);		   
	if(DCMI_SingleRandomWrite(0x71, 0x80)==0) //color test	on
		printf("color test on..\r\n");
	else
		printf("color test on fails!!\r\n");



	Delay(1000);
	 		
//	DCMI_SingleRandomWrite(0x70, 0x80);		   
	if(DCMI_SingleRandomWrite(0x71, 0x00)==0) //color test off
		printf("color test off..\r\n");
	else
		printf("color test off fails!!\r\n");
			  		 
	while (1)
	{
		Delay(1000);
		fpsbuff[i]=fps;
  		fps=0;
		i++;
		if(i==5)
		{
			disfps=(fpsbuff[0]+fpsbuff[1]+fpsbuff[2]+fpsbuff[3]+fpsbuff[4])/5;
			printf("FPS:%d\r\n",disfps);

			i=0;
		}	 	   		
//	sprintf((char* )lcd_buff1,"FPS:%d  ",disfps);
//			GUI_Text(0,0,lcd_buff1,White,Black);
	}
}

void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0)
  {}
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void LCD_CS_ResetBits(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//LCD_CS PD7

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	


	GPIO_ResetBits(GPIOD , GPIO_Pin_7);		 //CS=0;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
