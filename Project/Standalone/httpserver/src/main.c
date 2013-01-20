/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
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
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "httpd.h"
#include "serial_debug.h"
#include "stm324xg_eval_tsc.h"

extern uint8_t RxBuffer[20];
extern __IO uint16_t RxCounter;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "  STM32F4Discovery  "
#define MESSAGE2   "       ExtBoard     "
#define MESSAGE3   "   Webserver Demo   "
#define MESSAGE4   "  www.taotaotech.cn "

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

/* Private function prototypes -----------------------------------------------*/
void LCD_LED_Init(void);
void NVIC_USART_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured to 
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  
	//add a long delay wait for DP83848 finish reset  
	int x, y;
	unsigned int i,j;
	int ethernetok = 0;
	char szTemp[21];
	for(i=0;i<50;i++)
	{
		for(j=0;j < 65500;j++);
	} 
#ifdef SERIAL_DEBUG
  DebugComPort_Init();
#endif
	NVIC_USART_Config();

  TSC_Init();                           /* Touchscreen Controller Init        */

  /*Initialize LCD and Leds */ 
  LCD_LED_Init();

	LCD_SetDisplayWindow(0, 0, 320, 240);
	LCD_WriteRAM_Prepare();
#if 1
	{
		int i;
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0xf800);
		}
	for(i=0;i<500;i++)
	{
		for(j=0;j < 65500;j++);
	} 
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0x07E0);
		}
	for(i=0;i<500;i++)
	{
		for(j=0;j < 65500;j++);
	} 
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0x001f);
		}
	for(i=0;i<500;i++)
	{
		for(j=0;j < 65500;j++);
	} 
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0x0000);
		}
	for(i=0;i<500;i++)
	{
		for(j=0;j < 65500;j++);
	} 
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0xffff);
		}
	for(i=0;i<500;i++)
	{
		for(j=0;j < 65500;j++);
	} 
	for (j = 0; j < 500; j++)
	{
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(i);
		}
	}
		for (i = 0; i <	320 * 240; i++)
		{
			LCD_WriteRAM(0x0000);
		}
	}
#endif
  printf("led init ok");
  
	LCD_DisplayStringLine(Line4, (uint8_t *)"Init BSP...");
  /* configure ethernet (GPIOs, clocks, MAC, DMA) */ 
  ethernetok = ETH_BSP_Config();
 
  if (ethernetok)
  {
  	LCD_DisplayStringLine(Line4, (uint8_t *)"Init LwIP...");
  	/* Initilaize the LwIP stack */
  	LwIP_Init();
  
	LCD_DisplayStringLine(Line4, (uint8_t *)"Init server...");
  
  	/* Http webserver Init */
  	httpd_init();
  	printf("httpd init ok");
	LCD_DisplayStringLine(Line4, (uint8_t *)"                ");
  }
  else
  {
    LCD_SetTextColor(Red);
	LCD_DisplayStringLine(Line6, (uint8_t *)" ETHERNET INIT  ");
	LCD_DisplayStringLine(Line7, (uint8_t *)"    FAILED!!!   ");
    LCD_SetTextColor(White);
  }
    
  /* Infinite loop */
  while (1)
  {  
	if (TSC_TouchDet()) {           /* Show touch screen activity         */
		TP_GetAdXY(&x, &y);
	}
	else {
	  x = 0;
	  y = 0;
	}
	sprintf(szTemp, "X:%04d Y:%04d       ", x, y);
	LCD_DisplayStringLine(Line4, (uint8_t *)szTemp);
	for (i = 0; i < RxCounter && i < 20; i++) szTemp[i] = RxBuffer[i];
	for (; i < 20; i++) szTemp[i] = ' ';
	szTemp[i] = 0;
	LCD_DisplayStringLine(Line5, (uint8_t *)szTemp);

	if (ethernetok)
	{
	    /* check if any packet received */
	    if (ETH_CheckFrameReceived())
	    { 
	      /* process received ethernet packet */
	      LwIP_Pkt_Handle();
	    }
	    /* handle periodic timers for LwIP */
	    LwIP_Periodic_Handle(LocalTime);
	}
  }   
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime)
  {     
  }
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
  * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
void LCD_LED_Init(void)
{
#ifdef USE_LCD
  /* Initialize the STM324xG-EVAL's LCD */
  STM324xG_LCD_Init();
#endif

  /* Initialize STM324xG-EVAL's LEDs */
  STM_EVAL_LEDInit(LED1);
  STM_EVAL_LEDInit(LED2);
  
#ifdef USE_LCD
  /* Clear the LCD */
  LCD_Clear(Black);

  /* Set the LCD Back Color */
  LCD_SetBackColor(Black);

  /* Display message on the LCD*/
  LCD_SetTextColor(White);
  LCD_DisplayStringLine(Line0, (uint8_t*)MESSAGE1);
  LCD_SetTextColor(Red);
  LCD_DisplayStringLine(Line1, (uint8_t*)MESSAGE2);
  LCD_SetTextColor(Green);
  LCD_DisplayStringLine(Line2, (uint8_t*)MESSAGE3);
  LCD_SetTextColor(Blue);
  LCD_DisplayStringLine(Line3, (uint8_t*)MESSAGE4);  

  /* Set the LCD Text Color */
  LCD_SetTextColor(White);
#endif
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_USART_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(EVAL_COM1, USART_IT_RXNE, ENABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
  {}
}
#endif


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
