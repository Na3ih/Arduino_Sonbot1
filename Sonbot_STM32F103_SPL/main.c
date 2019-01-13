/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"
#include "stm32f1xx_nucleo.h"

#include "STM32_L298N_SPL.h"

/* - - - - - - - PINOUT - - - - - - - - */
static const uint16_t ena_pin = GPIO_Pin_11;
static const uint16_t in1_pin = GPIO_Pin_10;
static const uint16_t in2_pin = GPIO_Pin_1;

static GPIO_TypeDef * ena_port = GPIOB;
static GPIO_TypeDef * in1_port = GPIOB;
static GPIO_TypeDef * in2_port = GPIOB;

static const uint16_t enb_pin = GPIO_Pin_0;
static const uint16_t in3_pin = GPIO_Pin_7;
static const uint16_t in4_pin = GPIO_Pin_6;

static GPIO_TypeDef * enb_port = GPIOB;
static GPIO_TypeDef * in3_port = GPIOA;
static GPIO_TypeDef * in4_port = GPIOA;

static const uint16_t servo_pin = GPIO_Pin_6;
static GPIO_TypeDef * servo_port = GPIOB;

static const uint16_t echo_pin1 = GPIO_Pin_2;
static const uint16_t echo_pin2 = GPIO_Pin_3;
static const uint16_t trig_pin = GPIO_Pin_1;
static GPIO_TypeDef * sonar_port = GPIOA;

/* - - - - - - - TIME COUNT - - - - - - - - */
int tim = 0;
void delay(int tim_ms);

/* - - - - 	SERVO FUNCTIONS	 - - - - - */
void ServoLeft(void);
void ServoRight(void);
void ServoMiddle(void);

/* - - - -  Helper Functions - - - - - - - */
uint8_t TryLeft(void);
uint8_t TryRight(void);

/* - - - -SONAR FUNCTIONS  - - - - - -	*/
volatile int START = 0, STOP = 0, MeasuredFlag = 0;		// time of HIGH at ECHO
float GetDistance_cm();

/*  - - - - - - - CONFIGURATION - - - - - - - */
void RCC_Configuration(void);
void GPIO_Configuration(void);
void PWM_Servo_Configuration(void);		// TIM4
void Sonar_Timer_Configuration(void);	// TIM2
	void PWM_TRIG_Configuration(void);
	void ECHO_Configuration(void);
	void NVIC_Configuration(void);		// for ECHO

/* - - - - - - - - Constant Values - - - - - - */
static const int ninety_deg_rotation_time = 200;		// [ms] 90* rotation, set for your platform
static const float robot_length = 10.0;			// distance to obstacle;    (set for your platform)


/* ==================================================================== */
/* =========================== MAIN ==================================== */

int main(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	PWM_Servo_Configuration();
	Sonar_Timer_Configuration();

	Motors_Init(ena_pin, ena_port, in1_pin, in1_port, in2_pin, in2_port,
	    		enb_pin, enb_port, in3_pin, in3_port, in4_pin, in4_port);

	ServoMiddle();

	float dist = 0.0;
	int state = 0;

	while(1)
	{

		if(MeasuredFlag)
		{
			dist = GetDistance_cm();
			
			if(dist > robot_length) GPIO_SetBits(GPIOC, GPIO_Pin_13);
			else GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		}

		switch(state)
		{
			case 0:	// GO
			
				if(dist <= robot_length) state = 1;
				else GoForward();
				break;

			case 1:	// STOP

				Brake();
				ServoMiddle();

				state = 2;
				break;

			case 2:	// TRY RIGHT

				if(TryRight()) state = 3;
				else state = 4;
				break;

			case 3:	// TURN RIGHT
				ServoMiddle();
				TurnRight();
				delay(ninety_deg_rotation_time);
				state = 0;
				break;

			case 4: // TRY LEFT

				if(TryLeft()) state = 5;
				else state = 6;
				break;

			case 5: // TURN LEFT


				TurnLeft();
				delay(ninety_deg_rotation_time);
				ServoMiddle();
				state = 0;
				break;

			case 6: // TURN AROUND

				TurnLeft();
				delay(2*ninety_deg_rotation_time);
				ServoMiddle();
				state = 0;
				break;
		}
	}

	return 0;
}


/* ===================================================================== */

// - - - - - - - - PERIPHERIALS CONFIGURATION - - - - - - - */
void PWM_Servo_Configuration(void)
{
	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Prescaler = 72 - 1;			// 72M / 72 = 1MHz
	timer.TIM_Period = 20000 - 1;		// 1000000 / 20000 = 50Hz -> T = 20ms
	TIM_TimeBaseInit(TIM4, &timer);

	TIM_OCInitTypeDef tim_chanel;
	TIM_OCStructInit(&tim_chanel);
	tim_chanel.TIM_OCMode = TIM_OCMode_PWM1;
	tim_chanel.TIM_OutputState = TIM_OutputState_Enable;
	tim_chanel.TIM_Pulse = 1500;	// 1.5ms
	tim_chanel.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &tim_chanel);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);		// buforowanie kana³u 1
	TIM_ARRPreloadConfig(TIM4, ENABLE);						// buforowanie licznika

	TIM_Cmd(TIM4, ENABLE);
}

void PWM_TRIG_Configuration(void)
{
	TIM_OCInitTypeDef pwm;
	TIM_OCStructInit(&pwm);
	pwm.TIM_OCMode = TIM_OCMode_PWM1;
	pwm.TIM_OutputState = TIM_OutputState_Enable;
	pwm.TIM_OCPolarity = TIM_OCPolarity_High;
	pwm.TIM_Pulse = 10;								// 10us
	TIM_OC2Init(TIM2, &pwm);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
}

void ECHO_Configuration(void)
{
	TIM_ICInitTypeDef ch1;
	TIM_ICStructInit(&ch1);
	ch1.TIM_Channel = TIM_Channel_3;
	ch1.TIM_ICFilter = 0;									// wy³¹czony
	ch1.TIM_ICPolarity = TIM_ICPolarity_Rising;				// narastaj¹ce zbocze na ECHO
	ch1.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch1.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(TIM2, &ch1);

	TIM_ICInitTypeDef ch2;
	TIM_ICStructInit(&ch2);
	ch2.TIM_Channel = TIM_Channel_4;
	ch2.TIM_ICFilter = 0;
	ch2.TIM_ICPolarity = TIM_ICPolarity_Falling;			//opadaj¹ce zbocze na ECHO
	ch2.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch2.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(TIM2, &ch2);

	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);					// W£¥CZENIE PRZERWAÑ OD KANALU 3 I 4
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);					// TIMERA 2
}

void Sonar_Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef timer;
	TIM_TimeBaseStructInit(&timer);
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_Prescaler = 72 - 1;					// 72MHz / 72 = 1MHz
	timer.TIM_Period = 50000;						// 50ms
	TIM_TimeBaseInit(TIM2, &timer);

	PWM_TRIG_Configuration();
	ECHO_Configuration();
	NVIC_Configuration();

	TIM_Cmd(TIM2, ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef interrupts;
	interrupts.NVIC_IRQChannel = TIM2_IRQn;
	interrupts.NVIC_IRQChannelCmd = ENABLE;
	interrupts.NVIC_IRQChannelPreemptionPriority = 0;
	interrupts.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&interrupts);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		START = TIM_GetCapture3(TIM2);
	}

	else if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		STOP = TIM_GetCapture4(TIM2);
		MeasuredFlag = 1;
	}
}

void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
	SysTick_Config(SystemCoreClock / 1000);
}

void SysTick_Handler()
{
	if(tim) --tim;
}

void delay(int time_ms)
{
	tim = time_ms;
	while(tim);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = servo_pin;			// B6 - PWM to servo
	GPIO_Init(servo_port, &gpio);

	gpio.GPIO_Pin = trig_pin;		// TIM2CH2,
	GPIO_Init(sonar_port, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = echo_pin1 | echo_pin2;		// TIM2 Ch3, Ch4
	GPIO_Init(sonar_port, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_13;				// LED
	GPIO_Init(GPIOC, &gpio);
}

/* - - - - - - - - - -  SERVO FUNCTIONS - - - - - - - - - */
void ServoLeft(void)
{
	TIM_SetCompare1(TIM4, 2000);
	delay(250);
}

void ServoRight(void)
{
	TIM_SetCompare1(TIM4, 600);
	delay(250);
}

void ServoMiddle(void)
{
	TIM_SetCompare1(TIM4, 1200);
	delay(250);
}

/* - - - -  SONAR FUNCTIONS  - - - - - -	*/

float GetDistance_cm(void)
{
	return (STOP - START) / 58.0;
}

/* - - - -  Helper Functions - - - - - - - */
uint8_t TryLeft(void)
{
	ServoLeft();
	delay(300);

	float distance = GetDistance_cm();

	ServoMiddle();

	if(distance > robot_length) return 1;
	else return 0;
}

uint8_t TryRight(void)
{
	ServoRight();

	float distance = GetDistance_cm();

	if(distance > robot_length) return 1;
	else return 0;
}
