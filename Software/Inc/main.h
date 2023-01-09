#ifndef __MAIN_H__
#define __MAIN_H__


#include "stm32f0xx_ll_iwdg.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"
#include "math.h"
#include "stdio.h"


#define MISO_Pin LL_GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin LL_GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define CAL_Pin LL_GPIO_PIN_0
#define CAL_GPIO_Port GPIOA
#define CLOSE_Pin LL_GPIO_PIN_1
#define CLOSE_GPIO_Port GPIOA
#define SET2_Pin LL_GPIO_PIN_2
#define SET2_GPIO_Port GPIOA
#define SET1_Pin LL_GPIO_PIN_3
#define SET1_GPIO_Port GPIOA
#define CLKIN_Pin LL_GPIO_PIN_0
#define CLKIN_GPIO_Port GPIOB
#define CLKIN_EXTI_IRQn EXTI0_1_IRQn
#define DIRIN_Pin LL_GPIO_PIN_1
#define DIRIN_GPIO_Port GPIOB
#define DIRIN_EXTI_IRQn EXTI0_1_IRQn
#define ENIN_Pin LL_GPIO_PIN_2
#define ENIN_GPIO_Port GPIOB
#define ENIN_EXTI_IRQn EXTI2_3_IRQn
#define LED_Pin LL_GPIO_PIN_11  
#define LED_GPIO_Port GPIOB
#define PWM1_Pin LL_GPIO_PIN_4
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin LL_GPIO_PIN_5
#define PWM2_GPIO_Port GPIOB
#define IN1_Pin LL_GPIO_PIN_6
#define IN1_GPIO_Port GPIOB
#define IN2_Pin LL_GPIO_PIN_7
#define IN2_GPIO_Port GPIOB
#define IN3_Pin LL_GPIO_PIN_8
#define IN3_GPIO_Port GPIOB
#define IN4_Pin LL_GPIO_PIN_9
#define IN4_GPIO_Port GPIOB
#define NSS_Pin LL_GPIO_PIN_4
#define NSS_GPIO_Port GPIOA


#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif



#define CAL     LL_GPIO_IsInputPinSet(CAL_GPIO_Port,  CAL_Pin) 
#define CLOSE   LL_GPIO_IsInputPinSet(CLOSE_GPIO_Port, CLOSE_Pin) 
#define SET1    LL_GPIO_IsInputPinSet(SET1_GPIO_Port, SET1_Pin) 
#define SET2    LL_GPIO_IsInputPinSet(SET2_GPIO_Port, SET2_Pin)
#define ENIN    LL_GPIO_IsInputPinSet(ENIN_GPIO_Port, ENIN_Pin)
#define DIRIN   LL_GPIO_IsInputPinSet(DIRIN_GPIO_Port,DIRIN_Pin)

#define LED_H     LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin)  
#define LED_L     LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin) 
#define LED_F     LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define IN1_HIGH  LL_GPIO_SetOutputPin(IN1_GPIO_Port, IN1_Pin) 
#define IN1_LOW   LL_GPIO_ResetOutputPin(IN1_GPIO_Port, IN1_Pin)
#define IN2_HIGH  LL_GPIO_SetOutputPin(IN2_GPIO_Port, IN2_Pin) 
#define IN2_LOW   LL_GPIO_ResetOutputPin(IN2_GPIO_Port, IN2_Pin)
#define IN3_HIGH  LL_GPIO_SetOutputPin(IN3_GPIO_Port, IN3_Pin) 
#define IN3_LOW   LL_GPIO_ResetOutputPin(IN3_GPIO_Port, IN3_Pin)
#define IN4_HIGH  LL_GPIO_SetOutputPin(IN4_GPIO_Port, IN4_Pin) 
#define IN4_LOW   LL_GPIO_ResetOutputPin(IN4_GPIO_Port, IN4_Pin)

#define NSS_H   LL_GPIO_SetOutputPin(NSS_GPIO_Port, NSS_Pin)  
#define NSS_L   LL_GPIO_ResetOutputPin(NSS_GPIO_Port, NSS_Pin) 

#define SPI_TX_OD  LL_GPIO_SetPinOutputType(MOSI_GPIO_Port, MOSI_Pin, LL_GPIO_OUTPUT_OPENDRAIN)
#define SPI_TX_PP  LL_GPIO_SetPinOutputType(MOSI_GPIO_Port, MOSI_Pin, LL_GPIO_OUTPUT_PUSHPULL)


/* SPI command for TLE5012 */
#define READ_STATUS				0x8001			//8000
#define READ_ANGLE_VALUE		0x8021			//8020
#define READ_SPEED_VALUE		0x8031			//8030

#define WRITE_MOD1_VALUE		0x5060			//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001

#define WRITE_MOD2_VALUE		0x5080			//0_1010_0_001000_0001
#define MOD2_VALUE	0x0800

#define WRITE_MOD3_VALUE		0x5091			//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000

#define WRITE_MOD4_VALUE		0x50E0			//0_1010_0_001110_0001
#define MOD4_VALUE	0x0098						//9bit 512

#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 0x000D
/* Functionality mode */
#define REFERESH_ANGLE		0

#define UMAXCL   250 
#define UMAXOP   150
#define UMAXSUM  25600  


extern int16_t kp;     
extern int16_t ki;
extern int16_t kd;

extern const uint8_t LPFA; 
extern const uint8_t LPFB;


extern int32_t s;
extern int32_t s_1;
extern int32_t s_sum;
extern int32_t r;   
extern int32_t r_1;   
extern uint8_t dir; 
extern int16_t y;   
extern int16_t y_1;
extern int32_t yw;  
extern int32_t yw_1;
extern int16_t advance;
extern int32_t wrap_count; 
extern int32_t e;  
extern int32_t iterm;
extern int32_t dterm;
extern int16_t u;     
extern int32_t stepnumber;
extern uint8_t stepangle;

extern uint16_t hccount;
extern uint8_t closemode;
extern uint8_t enmode;

extern void Output(int32_t theta,uint8_t effort);
extern uint16_t ReadValue(uint16_t RegValue);
extern uint16_t ReadAngle(void);

static void LL_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);

void SetModeCheck(void);
void UsDelay(uint16_t us);
void Output(int32_t theta,uint8_t effort);
int16_t Mod(int32_t xMod,int16_t mMod);
void OneStep(void);
void FlashUnlock(void);
void FlashLock(void);
uint8_t FlashGetStatus(void);
uint8_t FlashWaitDone(uint16_t time);
uint8_t FlashErasePage(uint32_t paddr);
void FlashErase32K(void);
uint8_t FlashWriteHalfWord(uint32_t faddr,uint16_t dat);
uint16_t FlashReadHalfWord(uint32_t faddr);
int fputc(int c,FILE *stream); 
int fgetc(FILE *stream); 
void SerialCheck(void);
uint16_t ReadValue(uint16_t RegValue);
void WriteValue(uint16_t RegAdd,uint16_t RegValue);
uint16_t ReadState(void);
uint16_t ReadAngle(void);
void CalibrateEncoder(void);


#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif 
