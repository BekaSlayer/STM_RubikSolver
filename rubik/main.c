#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h";
#include "stm32f4xx.h"

/* PINS:
 * USART3: PC10 = Tx, PC11 = Rx
 * TIM3 channel 1 (PB4)
 * TIM3 channel 2 (PB5)
 * TIM3 channel 3 (PC8)
 * TIM3 channel 1 (PB6)
 * TIM3 channel 2 (PB7)
 * TIM3 channel 3 (PB8)
 */

#define POS_NEUTRAL 1499
#define POS_CLOCKWISE 2299
#define POS_CCLOCKWISE 699
#define ROTATION_DELAY 180
#define MOTOR_TOP TIM3->CCR1
#define MOTOR_FRONT TIM3->CCR2
#define MOTOR_LEFT TIM3->CCR3
#define MOTOR_RIGHT TIM4->CCR1
#define MOTOR_BACK TIM4->CCR2
#define MOTOR_BOTTOM TIM4->CCR3
int TIM2_COUNT; // ms count for delay function
char ACTION = 'n';

void delay(int ms) {
	TIM2_COUNT = 0;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_Cmd(TIM2, ENABLE);
	while(TIM2_COUNT < ms);
	TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler(void) {
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM2_COUNT++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

/* receives data via bluetooth */
void USART3_IRQHandler() {
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		ACTION = USART3->DR; // receive character
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}

void enable_clocks() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void GPIO_init() {
	// pins B4 to B8
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// pin C8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// set alternate functions for pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
}

/* Pins for USART3: PC10 = Tx, PC11 = Rx */
void USART_init() {
	// set up USART3
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	// set up USART interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(USART3_IRQn);
}

/* timer tick frequency = 84000000Hz */
void TIMER_init() {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	// TIM2 controls delay funtions (1 timer cycle = 1ms)
	TIM_BaseStruct.TIM_Prescaler =  83;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 999;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
	// set up timer interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	// TIM3 and TIM4 frequency set to 50Hz
	// TIM3 for controlling 3 servos
	TIM_BaseStruct.TIM_Prescaler =  83;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 19999;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
	TIM_Cmd(TIM3, ENABLE);

	// TIM4 for controlling 3 servos
	TIM_BaseStruct.TIM_Prescaler =  83;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = 19999;
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	TIM_Cmd(TIM4, ENABLE);
}

void PWM_init() {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	// TIM3 channel 1 (PB4)
	TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	// TIM3 channel 2 (PB5)
	TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	// TIM3 channel 3 (PC8)
	TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	// TIM4 channel 1 (PB6)
	TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    // TIM4 channel 2 (PB7)
    TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    // TIM4 channel 3 (PB8)
    TIM_OCInitStructure.TIM_Pulse = POS_NEUTRAL;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
}

void init() {
	SystemInit();
	enable_clocks();
	GPIO_init();
	USART_init();
	TIMER_init();
	PWM_init();
}

int main() {
	init();
	while(1) {
		switch(ACTION) {
		case 'e':
			MOTOR_TOP = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 'r':
			MOTOR_TOP = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case 'd':
			MOTOR_FRONT = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 'f':
			MOTOR_FRONT = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case 'a':
			MOTOR_LEFT = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 's':
			MOTOR_LEFT = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case 'g':
			MOTOR_RIGHT = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 'h':
			MOTOR_RIGHT = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case 'j':
			MOTOR_BACK = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 'k':
			MOTOR_BACK = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case 'x':
			MOTOR_BOTTOM = POS_CCLOCKWISE;
			ACTION = 'n';
			break;
		case 'c':
			MOTOR_BOTTOM = POS_CLOCKWISE;
			ACTION = 'n';
			break;

		case ' ':
			MOTOR_TOP = POS_NEUTRAL;
			MOTOR_FRONT = POS_NEUTRAL;
			MOTOR_RIGHT = POS_NEUTRAL;
			MOTOR_BACK = POS_NEUTRAL;
			MOTOR_LEFT = POS_NEUTRAL;
			MOTOR_BOTTOM = POS_NEUTRAL;
			break;
		}
	}
}
