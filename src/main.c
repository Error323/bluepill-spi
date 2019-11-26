#include "main.h"

#define CRC16_CCITT 0x1021
#define SPI_MSG_LEN 7

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/* Buffer used for transmission */
uint16_t aTxBuffer[SPI_MSG_LEN];
uint8_t ubNbDataToTransmit = SPI_MSG_LEN;
__IO uint8_t ubTransmissionComplete = 0;

/* Buffer used for reception */
uint16_t aRxBuffer[SPI_MSG_LEN];
uint8_t ubNbDataToReceive  = SPI_MSG_LEN + 1;
__IO uint8_t ubReceptionComplete = 0;

/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);
void     Configure_DMA(void);
void     Configure_SPI(void);
void     Activate_SPI(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Blinking(uint32_t Period);
void     LED_Off(void);
void     WaitAndCheckEndOfTransfer(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 72 MHz */
  SystemClock_Config();

  /* Initialize LED2 */
  LED_Init();

  /* Configure the SPI2 parameters */
  Configure_SPI();

  /* Configure DMA channels for the SPI2  */
  Configure_DMA();

  /* Enable the SPI2 peripheral */
  Activate_SPI();

  /* Wait for the end of the transfer and check received data */
  /* LED blinking FAST during waiting time */
  WaitAndCheckEndOfTransfer();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief This function configures the DMA Channels for SPI2
  * @note  This function is used to :
  *        -1- Enable DMA1 clock
  *        -2- Configure NVIC for DMA1 transfer complete/error interrupts
  *        -3- Configure the DMA1_Channel4 functional parameters
  *        -4- Configure the DMA1_Channel5 functional parameters
  *        -5- Enable DMA1 interrupts complete/error
  * @param   None
  * @retval  None
  */
void Configure_DMA(void)
{
  /* DMA1 used for SPI2 Transmission
   * DMA1 used for SPI2 Reception
   */
  /* (1) Enable the clock of DMA1 and DMA1 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

 /* (2) Configure NVIC for DMA transfer complete/error interrupts */
  NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

  /* (3) Configure the DMA1_Channel4 functional parameters */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_4,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                        LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_4,
                         LL_SPI_DMA_GetRegAddr(SPI2), (uint32_t)aRxBuffer,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, ubNbDataToReceive);



  /* (4) Configure the DMA1_Channel5 functional parameters */
  LL_DMA_ConfigTransfer(DMA1,
                        LL_DMA_CHANNEL_5,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
                        LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, (uint32_t)aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI2),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ubNbDataToTransmit);


  /* (5) Enable DMA interrupts complete/error */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
}

/**
  * @brief This function configures SPI2.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI2 pins.
  *        -2- Configure SPI2 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_SPI(void)
{
  /* (1) Enables GPIO clock and configures the SPI2 pins ********************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* Configure SCK Pin connected to pin 30 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_DOWN);

  /* Configure MISO Pin connected to pin 28 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_DOWN);

  /* Configure MOSI Pin connected to pin 26 of CN10 connector */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);

  /* (2) Configure SPI2 functional parameters ********************************/
  /* Enable the peripheral clock of GPIOB */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  /* Configure SPI2 communication */
  LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV32);
  LL_SPI_SetTransferDirection(SPI2,LL_SPI_FULL_DUPLEX);
  LL_SPI_SetClockPhase(SPI2, LL_SPI_PHASE_1EDGE);
  LL_SPI_SetClockPolarity(SPI2, LL_SPI_POLARITY_LOW);
  /* Reset value is LL_SPI_MSB_FIRST */
  LL_SPI_SetTransferBitOrder(SPI2, LL_SPI_MSB_FIRST);
  LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_16BIT);
  LL_SPI_SetNSSMode(SPI2, LL_SPI_NSS_SOFT);
  /* Set SPI2 CRC Polynomial to XModem 16 bit */
  LL_SPI_SetCRCPolynomial(SPI2, CRC16_CCITT);
  LL_SPI_SetMode(SPI2, LL_SPI_MODE_SLAVE);

  /* Configure SPI2 DMA transfer interrupts */
  /* Enable DMA RX Interrupt */
  LL_SPI_EnableDMAReq_RX(SPI2);
  /* Enable DMA TX Interrupt */
  LL_SPI_EnableDMAReq_TX(SPI2);
}

/**
  * @brief  This function Activate SPI2
  * @param  None
  * @retval None
  */
void Activate_SPI(void)
{
  /* Enable CRC for SPI2 */
  LL_SPI_EnableCRC(SPI2);

  /* Enable SPI2 */
  LL_SPI_Enable(SPI2);

  /* Enable DMA Channels */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

/**
  * @brief  Initialize LED2.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LED2_GPIO_CLK_ENABLE();

  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LED_Off();
}

/**
  * @brief  Turn-on LED2.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
  * @param  Period : Period of time (in ms) between each toggling of LED
  *   This parameter can be user defined values. Pre-defined values used in that example are :
  *     @arg LED_BLINK_FAST : Fast Blinking
  *     @arg LED_BLINK_SLOW : Slow Blinking
  *     @arg LED_BLINK_ERROR : Error specific Blinking
  * @retval None
  */
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}

/**
  * @brief  Wait end of transfer and check if received Data are well.
  * @param  None
  * @retval None
  */
void WaitAndCheckEndOfTransfer(void)
{
  /* 1 - Wait end of transmission */
  while (ubTransmissionComplete != 1)
  {
  }
  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  /* 2 - Wait end of reception */
  while (ubReceptionComplete != 1)
  {
  }
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  /* Turn On Led if data are well received */
  LED_On();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Enable HSE oscillator */
  //LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 72MHz */
  LL_Init1msTick(72000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(72000000);
}
/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
  * @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1_ReceiveComplete_Callback(void)
{
  /* DMA Rx transfer completed */
  ubReceptionComplete = 1;
}

/**
  * @brief  Function called from DMA1 IRQ Handler when Tx transfer is completed
  * @param  None
  * @retval None
  */
void DMA1_TransmitComplete_Callback(void)
{
  /* DMA Tx transfer completed */
  ubTransmissionComplete = 1;
}

/**
  * @brief  Function called in case of error detected in SPI IT Handler
  * @param  None
  * @retval None
  */
void SPI2_TransferError_Callback(void)
{
  /* Disable DMA1 Rx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

  /* Disable DMA1 Tx Channel */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  /* Set LED2 to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
