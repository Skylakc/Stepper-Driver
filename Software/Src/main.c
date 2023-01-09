#include "main.h"
#include "sin_table.h"

int16_t kp=30;     
int16_t ki=10;  
int16_t kd=100;  //pid parameters

const uint8_t LPFA=125; 
const uint8_t LPFB=3;

int32_t s=0;
int32_t s_1=0;
int32_t s_sum=0;
int32_t r=0; 
int32_t r_1=0;
uint8_t dir=1; 
int16_t y=0; 
int16_t y_1=0;
int32_t yw=0;  
int32_t yw_1=0;
int16_t advance=0;
int32_t wrap_count=0; 
int32_t e=0; 
int32_t iterm=0;
int32_t dterm=0;
int16_t u=0;    
int32_t stepnumber=0;
uint8_t stepangle=0;

uint16_t hccount=0;
uint8_t closemode;
uint8_t enmode=1;
//variables

int main(void)
{
  LL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  LL_mDelay(100);
  SetModeCheck();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  while(1)
  { 	
	SerialCheck(); 
  }
}

//The NVIC Priority Setted at Each Init Function, Please Check it

static void LL_Init(void) //LL Library Init
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  NVIC_SetPriority(SVC_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0);
  NVIC_SetPriority(SysTick_IRQn, 0);
}


void SystemClock_Config(void) //SystemClock
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
    Error_Handler();  
  }
  LL_RCC_HSI_Enable();
  
  while(LL_RCC_HSI_IsReady() != 1)
  {}
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {}
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();
	  
  while(LL_RCC_PLL_IsReady() != 1)
  {}
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	  
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {}
  LL_Init1msTick(48000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  NVIC_SetPriority(SysTick_IRQn, 0);
}

static void MX_IWDG_Init(void) //Watch Dog Init
{
  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
  LL_IWDG_SetWindow(IWDG, 4095);
  LL_IWDG_SetReloadCounter(IWDG, 500);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {}
  LL_IWDG_ReloadCounter(IWDG);
}

static void MX_SPI1_Init(void) //SPI Init, the Communication Port Between MCU and Angle Sensor
{
  LL_SPI_InitTypeDef SPI_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
	
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  
  /**SPI1 GPIO Configuration  
  PA4   ------> SPI1_NSS
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI 
  */

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA); 
  LL_SPI_Enable(SPI1); 
}

static void MX_TIM1_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration  
  PA12   ------> TIM1_ETR 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFF;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV8_N6);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_EXT_MODE2);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  LL_TIM_EnableCounter(TIM1);
}

static void MX_TIM3_Init(void) //TIM3 Init,TIM3 Used to PWM Generate
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	
  /**TIM3 GPIO Configuration  
  PB4   ------> TIM3_CH1
  PB5   ------> TIM3_CH2 
  */
  GPIO_InitStruct.Pin = PWM1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(PWM1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(PWM2_GPIO_Port, &GPIO_InitStruct);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 256;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM3);

  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);

  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM3);
  
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM3);
}

static void MX_TIM6_Init(void) //TIM6 Init,TIM6 Used to Sweep Angle by TLE5012
{
  LL_TIM_InitTypeDef TIM_InitStruct;

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
  NVIC_SetPriority(TIM6_IRQn, 1);
  NVIC_EnableIRQ(TIM6_IRQn);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 4800;
  LL_TIM_Init(TIM6, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM6);
	
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);
}

static void MX_USART1_UART_Init(void) //USART Init
{
  LL_USART_InitTypeDef USART_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);

  LL_USART_DisableIT_CTS(USART1);
  LL_USART_DisableOverrunDetect(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
}

static void MX_GPIO_Init(void) //GPIO Init
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_EXTI_InitTypeDef EXTI_InitStruct;

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
  LL_GPIO_ResetOutputPin(IN1_GPIO_Port, IN1_Pin);
  LL_GPIO_ResetOutputPin(IN2_GPIO_Port, IN2_Pin);
  LL_GPIO_ResetOutputPin(IN4_GPIO_Port, IN4_Pin);
  LL_GPIO_ResetOutputPin(IN3_GPIO_Port, IN3_Pin);


  GPIO_InitStruct.Pin = CAL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(CAL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CLOSE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(CLOSE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SET2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SET2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SET1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SET1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IN4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IN3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(IN3_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE2);
  
  LL_GPIO_SetPinPull(DIRIN_GPIO_Port, DIRIN_Pin, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinPull(ENIN_GPIO_Port, ENIN_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinMode(DIRIN_GPIO_Port, DIRIN_Pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinMode(ENIN_GPIO_Port, ENIN_Pin, LL_GPIO_MODE_INPUT);

  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  NVIC_SetPriority(EXTI0_1_IRQn, 0);
  NVIC_DisableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI2_3_IRQn, 1);
  NVIC_DisableIRQ(EXTI2_3_IRQn);
}

void SetModeCheck(void) //Working Mode Check, Selected by Bit Switch
{
  WriteValue(WRITE_MOD2_VALUE,MOD2_VALUE);
  uint16_t state=ReadState();
  if(state&0x0080)
  {
    for(uint8_t m=0;m<10;m++)
    {
      LED_H;
	  LL_mDelay(200);
	  LED_L;
	  LL_mDelay(200);	
    } 
  }	
  if(CAL==0)  
	CalibrateEncoder();
  
  if((SET1==1)&&(SET2==1)) //Electronic Gear, /2,/4,/8,/16
    stepangle=16;
  else if((SET1==0)&&(SET2==1))
    stepangle=8;
  else if((SET1==1)&&(SET2==0))
    stepangle=4;
  else
    stepangle=2;
  
  if(CLOSE==0) //Close Loop
  {
    closemode=1;
	r=*(volatile uint16_t*)((ReadValue(READ_ANGLE_VALUE)>>1)*2+0x08008000);
	s_sum=r;
    y=r;
    y_1=y;
	yw=y;  
	yw_1=yw;
  }
  else
  {
	closemode=0;
  }
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void Output(int32_t theta,uint8_t effort) //The PWM Generator Function to Drive Half-Bridge A4950
{	
  int16_t v_coil_A;
  int16_t v_coil_B;

  int16_t sin_coil_A;
  int16_t sin_coil_B;
	
  int16_t angle_1;
  int16_t angle_2;
		
  float phase_multiplier=12.5f;
	
  angle_1=Mod(phase_multiplier*theta,4096);   
  angle_2=angle_1+1024;
  if(angle_2>4096)
    angle_2-=4096;
  
  sin_coil_A=sin_1[angle_1];
  sin_coil_B=sin_1[angle_2];

  v_coil_A=effort*sin_coil_A/1024;
  v_coil_B=effort*sin_coil_B/1024;
	
  if(v_coil_A>=0)  
  {
    LL_TIM_OC_SetCompareCH2(TIM3,v_coil_A);  
	IN1_HIGH;  
    IN2_LOW;  
  }
  else  
  {
    LL_TIM_OC_SetCompareCH2(TIM3,-v_coil_A);  
	IN1_LOW;     
    IN2_HIGH;  
  } 
  if(v_coil_B>=0)  
  {
    LL_TIM_OC_SetCompareCH1(TIM3,v_coil_B);  
	IN3_HIGH;  
    IN4_LOW;  
  }
  else 
  {
    LL_TIM_OC_SetCompareCH1(TIM3,-v_coil_B); 
	IN3_LOW;     
    IN4_HIGH;    
  }
}

void OneStep(void)
{          
  if(dir) 
    stepnumber+=1;
  else 
    stepnumber-=1;
  
  Output(81.92f*stepnumber,80);
  LL_mDelay(10);
}

int16_t Mod(int32_t xMod,int16_t mMod) 
{
  int16_t temp;
  temp=xMod%mMod;
  if(temp<0)
    return (temp+mMod);
  else
	return  temp;
}

void UsDelay(uint16_t us) //Precise Delay Function
{
  us*=10;
  while(us)	
  {
    __NOP();
	__NOP();
	us--;
  }
}

void FlashUnlock(void)
{
  FLASH->KEYR=FLASH_KEY1;
  FLASH->KEYR=FLASH_KEY2;
}

void FlashLock(void)
{
  FLASH->CR|=1<<7; 
}

uint8_t FlashGetStatus(void)
{
  uint32_t res;
  res=FLASH->SR;
  if(res&(1<<0))
    return 1; 
  else if(res&(1<<2))
	return 2; 
  else if(res&(1<<4))
	return 3; 
  return 0;     
}

uint8_t FlashWaitDone(uint16_t time)
{
  uint8_t res;
  do
  {
    res=FlashGetStatus();
    if(res!=1)
	  break;
	UsDelay(1);	
    time--;
  }while(time);
  if(time==0)
    res=0xff;
  return res;
}

uint8_t FlashErasePage(uint32_t paddr)
{
  uint8_t res=0;
  res=FlashWaitDone(0X5FFF);
  if(res==0)
  {
    FLASH->CR|=1<<1; 
    FLASH->AR=paddr; 
    FLASH->CR|=1<<6; 
    res=FlashWaitDone(0X5FFF); 
    if(res!=1) 
      FLASH->CR&=~(1<<1); 
  }
  return res;
}

void FlashErase32K(void)
{
  FlashErasePage(0x08008000);
  FlashErasePage(0x08008400);
  FlashErasePage(0x08008800);
  FlashErasePage(0x08008C00);
  FlashErasePage(0x08009000);
  FlashErasePage(0x08009400);
  FlashErasePage(0x08009800);
  FlashErasePage(0x08009C00);
  FlashErasePage(0x0800A000);
  FlashErasePage(0x0800A400);
  FlashErasePage(0x0800A800);
  FlashErasePage(0x0800AC00);
  FlashErasePage(0x0800B000);
  FlashErasePage(0x0800B400);
  FlashErasePage(0x0800B800);
  FlashErasePage(0x0800BC00);
  FlashErasePage(0x0800C000);
  FlashErasePage(0x0800C400);
  FlashErasePage(0x0800C800);
  FlashErasePage(0x0800CC00);
  FlashErasePage(0x0800D000);
  FlashErasePage(0x0800D400);
  FlashErasePage(0x0800D800);
  FlashErasePage(0x0800DC00);
  FlashErasePage(0x0800E000);
  FlashErasePage(0x0800E400);
  FlashErasePage(0x0800E800);
  FlashErasePage(0x0800EC00);
  FlashErasePage(0x0800F000);
  FlashErasePage(0x0800F400);
  FlashErasePage(0x0800F800); 
  FlashErasePage(0x0800FC00);
}

uint8_t FlashWriteHalfWord(uint32_t faddr,uint16_t dat)
{
  uint8_t  res;
  res=FlashWaitDone(0XFF);
  if(res==0)
  {
    FLASH->CR|=1<<0;
    *(volatile uint16_t*)faddr=dat; 
    res=FlashWaitDone(0XFF);
	if(res!=1)
    {
      FLASH->CR&=~(1<<0); 
    }
  }
  return res;
}

uint16_t FlashReadHalfWord(uint32_t faddr)
{
  return *(volatile uint16_t*)faddr;
}

int fputc(int c,FILE *stream) 
{	   
  LL_USART_TransmitData8(USART1,c);
  while(!LL_USART_IsActiveFlag_TXE(USART1))__NOP();
  LL_USART_ClearFlag_TC(USART1); 
  return c;
}

int fgetc(FILE *stream) 
{
  while(!LL_USART_IsActiveFlag_RXNE(USART1)) __NOP();
  return ((char)LL_USART_ReceiveData8(USART1));
}

void SerialCheck(void) //PID Parameters Test by Serial
{        
  printf("----- Skylake Stepper -----\r\n");
  uint8_t quit=0;
  char command;
  while(!quit)
  {
    printf("Edit PID gains:\r\n");
    printf("p ----- kp =%d\r\n",kp);
    printf("i ----- ki =%d\r\n",ki);
    printf("d ----- kd =%d\r\n",kd);
    printf("q ----- quit\r\n");
	printf("Please input your command:\r\n"); 
	scanf("%c",&command);
	printf("Your input command is:%c\r\n",command);
    switch(command) 
	{
      case 'p':
      {
        printf("kp = ?\r\n");
		scanf("%hd",&kp);
        printf("new kp = %d\r\n",kp);
      }
      break;
      case 'i':
      {
        printf("ki = ?\r\n");
		scanf("%hd",&ki);
		printf("new ki = %d\r\n",ki);
      }
      break;
      case 'd':
      {
        printf("kd = ?\r\n");
		scanf("%hd",&kd);
        printf("new kd = %d\r\n",kd);
      }
      break;
      case 'q':
      {  
        quit=1;
        printf("done...\r\n");
      }
      default:
      {}
      break;
    }
  }
}

uint16_t ReadValue(uint16_t RegAdd)
{
  uint16_t data;
  NSS_L;
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,RegAdd);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  data=LL_SPI_ReceiveData16(SPI1);
  SPI_TX_OD;
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,0xFFFF);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  data=LL_SPI_ReceiveData16(SPI1)&0x7FFF;
  NSS_H;
  SPI_TX_PP;
  return data;
}

void WriteValue(uint16_t RegAdd,uint16_t RegValue)
{
  NSS_L;
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,RegAdd);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  LL_SPI_ReceiveData16(SPI1);
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,RegValue);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  LL_SPI_ReceiveData16(SPI1);
  NSS_H;
}

uint16_t ReadState(void)
{
  return (ReadValue(READ_STATUS));
}

uint16_t ReadAngle(void)
{
  return (ReadValue(READ_ANGLE_VALUE)>>1);
}

void CalibrateEncoder(void)  //The Angle Sensor Calibrate Function
{   
  int32_t encoderReading=0;    
  int32_t currentencoderReading=0;
  int32_t lastencoderReading=0;        

  int32_t iStart=0;    
  int32_t jStart=0;
  int32_t stepNo=0;
  
  int32_t fullStepReadings[200];
  int32_t ticks=0;	
  uint32_t address=0x08008000; //The Calibration Data Flash Address

  uint16_t lookupAngle;
		
  dir=1; 
  Output(0,80);//42--100 57--80
  for(uint8_t m=0;m<4;m++)
  {
    LED_H;
	LL_mDelay(250);
    LED_L;
	LL_mDelay(250);	
  } 
  for(int16_t x=0;x<=199;x++) 
  {    
    encoderReading=0;
   	LL_mDelay(20);                     
    lastencoderReading=ReadAngle();     
    for(uint8_t reading=0;reading<10;reading++) 
	{ 
      currentencoderReading=ReadAngle(); 
      if(currentencoderReading-lastencoderReading<-8192)
        currentencoderReading+=16384;
      else if(currentencoderReading-lastencoderReading>8192)
        currentencoderReading-=16384;
 
      encoderReading+=currentencoderReading;
      LL_mDelay(10);
      lastencoderReading=currentencoderReading;
    }
    encoderReading=encoderReading/10;
    if(encoderReading>16384)
      encoderReading-=16384;
    else if(encoderReading<0)
      encoderReading+=16384;
    fullStepReadings[x]=encoderReading;  
    OneStep();
	LL_mDelay(100); 
  }
  dir=0; 
  OneStep();
  LL_mDelay(1000); 
  for(int16_t x=199;x>=0;x--) 
  {    
    encoderReading=0;
   	LL_mDelay(20);                     
    lastencoderReading=ReadAngle();     
    for(uint8_t reading=0;reading<10;reading++) 
	{ 
      currentencoderReading=ReadAngle(); 
      if(currentencoderReading-lastencoderReading<-8192)
        currentencoderReading+=16384;
      else if(currentencoderReading-lastencoderReading>8192)
        currentencoderReading-=16384;
 
      encoderReading+=currentencoderReading;
      LL_mDelay(10);
      lastencoderReading=currentencoderReading;
    }
    encoderReading=encoderReading/10;
    if(encoderReading>16384)
      encoderReading-=16384;
    else if(encoderReading<0)
      encoderReading+=16384;
    fullStepReadings[x]=(fullStepReadings[x]+encoderReading)/2;  
    OneStep();
	LL_mDelay(100); 
  }
  LL_TIM_OC_SetCompareCH1(TIM3,0);  
  LL_TIM_OC_SetCompareCH2(TIM3,0); 
  for(uint8_t i=0;i<200;i++) 
  {
    ticks=fullStepReadings[(i+1)%200]-fullStepReadings[i%200];
    if(ticks<-15000) 
      ticks+=16384;
    else if(ticks>15000)	
	  ticks-=16384;	
    for(int32_t j=0;j<ticks;j++) 
	{
      stepNo=(fullStepReadings[i]+j)%16384;
      if(stepNo==0) 
      {
        iStart=i;
        jStart=j;
      }
    }
  }
  FlashUnlock();
  FlashErase32K();
  for(int32_t i=iStart;i<(iStart+200+1);i++) 
  {
	ticks=fullStepReadings[(i+1)%200]-fullStepReadings[i%200];
    if(ticks<-15000) 
      ticks+=16384;         
    if(i==iStart) 
	{ 
      for(int32_t j=jStart;j<ticks;j++) 
	  {
        lookupAngle=(8192*i+8192*j/ticks)%1638400/100;
		FlashWriteHalfWord(address,(uint16_t)lookupAngle);
		address+=2;
      }
    }
    else if(i==(iStart+200)) 
	{ 
      for(int32_t j=0;j<jStart;j++) 
	  {
        lookupAngle=((8192*i+8192*j/ticks)%1638400)/100;
		FlashWriteHalfWord(address,(uint16_t)lookupAngle);
		address+=2;
      }
    }
    else 
	{                        //this is the general case
      for(int32_t j=0;j<ticks;j++) 
      {
        lookupAngle=((8192*i+8192*j/ticks)%1638400)/100;
		FlashWriteHalfWord(address,(uint16_t)lookupAngle); //Write in Flash
		address+=2;
      }
    }
  }
  FlashLock();
  while(1)
  {
    LED_F;
	LL_mDelay(500);
  }
}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

