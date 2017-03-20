
#include "motor.h"

#define MT_DUTY_RANGE	3000
#define MAX_PWM_CH		2

#define MT0_DIR0_GPIO	GPIOA
#define MT0_DIR0_PIN	GPIO_PIN_3
#define MT0_DIR1_GPIO	GPIOA
#define MT0_DIR1_PIN	GPIO_PIN_4
#define MT0_PWM_GPIO	GPIOA
#define MT0_PWM_PIN		GPIO_PIN_0
#define MT0_PWM_CH		TIM_CHANNEL_1

#define MT1_DIR0_GPIO	GPIOA
#define MT1_DIR0_PIN	GPIO_PIN_5
#define MT1_DIR1_GPIO	GPIOA
#define MT1_DIR1_PIN	GPIO_PIN_6
#define MT1_PWM_GPIO	GPIOA
#define MT1_PWM_PIN		GPIO_PIN_1
#define MT1_PWM_CH		TIM_CHANNEL_2


static TIM_HandleTypeDef h_Tim2, h_Tim3;
static TIM_OC_InitTypeDef MT0_OC_Struct, MT1_OC_Struct;
static TIM_Encoder_InitTypeDef Enc_Struct;

static uint32_t pwm_period;

void MT_PWM_init(uint32_t pwm_freq)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable GPIO clock */
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN);

	/* GPIO Initialize */
	/* MT0 PWM Out:PA0 (Nucleo: A0) */
	GPIO_InitStruct.Pin 		= MT0_PWM_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF1_TIM2;
	HAL_GPIO_Init(MT0_PWM_GPIO, &GPIO_InitStruct);
	/* MT0 Direction Control:PA3 (Nucleo: A2) and PA4 (Nucleo: A3) */
	GPIO_InitStruct.Pin 		= MT0_DIR0_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(MT0_DIR0_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.Pin 		= MT0_DIR1_PIN;
	HAL_GPIO_Init(MT0_DIR1_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(MT0_DIR0_GPIO, MT0_DIR0_PIN, 0);
	HAL_GPIO_WritePin(MT0_DIR1_GPIO, MT0_DIR1_PIN, 0);
	
	/* MT1 PWM Out:PA1 (Nucleo: A1) */
	GPIO_InitStruct.Pin 		= MT1_PWM_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF1_TIM2;
	HAL_GPIO_Init(MT1_PWM_GPIO, &GPIO_InitStruct);
	/* MT0 Direction Control:PA5 (Nucleo: A4) and PA6 (Nucleo: A5) */
	GPIO_InitStruct.Pin 		= MT1_DIR0_PIN;
	GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(MT1_DIR0_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.Pin 		= MT1_DIR1_PIN;
	HAL_GPIO_Init(MT1_DIR1_GPIO, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(MT1_DIR0_GPIO, MT1_DIR0_PIN, 0);
	HAL_GPIO_WritePin(MT1_DIR1_GPIO, MT1_DIR1_PIN, 0);
	
	/* Timer Initialize */
	pwm_period = (uint32_t)(64000000.f / (float)pwm_freq - 1.f);
	
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN);
	h_Tim2.Instance = TIM2;
	h_Tim2.Init.Prescaler = 0;
	h_Tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	h_Tim2.Init.Period = pwm_period;
	h_Tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	HAL_TIM_Base_Init(&h_Tim2);
	
	if(HAL_TIM_PWM_Init(&h_Tim2) != HAL_OK){
		printf("Timer Initialize Error!\n");
		while(1);
	}	
	
	// Config PWM for MT0
	MT0_OC_Struct.OCMode = TIM_OCMODE_PWM1;
	MT0_OC_Struct.OCPolarity = TIM_OCPOLARITY_HIGH;
	MT0_OC_Struct.OCFastMode = TIM_OCFAST_DISABLE;
	MT0_OC_Struct.Pulse = 0;
	if(HAL_TIM_PWM_ConfigChannel(&h_Tim2, &MT0_OC_Struct, MT0_PWM_CH) != HAL_OK){
		printf("MT0 PWM Initialize Error!\n");
		while(1);
	}
	// Config PWM for MT1
	MT1_OC_Struct.OCMode = TIM_OCMODE_PWM1;
	MT1_OC_Struct.OCPolarity = TIM_OCPOLARITY_HIGH;
	MT1_OC_Struct.OCFastMode = TIM_OCFAST_DISABLE;
	MT1_OC_Struct.Pulse = 0;
	if(HAL_TIM_PWM_ConfigChannel(&h_Tim2, &MT1_OC_Struct, MT1_PWM_CH) != HAL_OK){
		printf("MT1 PWM Initialize Error!\n");
		while(1);
	}
	
	HAL_TIM_PWM_Start(&h_Tim2, MT0_PWM_CH); // Start PWM for MT0
	HAL_TIM_PWM_Start(&h_Tim2, MT1_PWM_CH); // Start PWM for MT1
}

void MT_PWM_setDuty(uint8_t ch, int16_t duty)
{
	uint32_t pulse;
	uint8_t dir_state;
	
	// CH limitation
	if(ch >= MAX_PWM_CH) ch = MAX_PWM_CH - 1;
	
	// duty range limitation
	if(duty > MT_DUTY_RANGE)	duty = MT_DUTY_RANGE;
	if(duty < -MT_DUTY_RANGE)	duty = -MT_DUTY_RANGE;
	
	// direction and brake control
	if(duty > 0){
		dir_state = 1; // Normal
		pulse = duty * (pwm_period / (float)MT_DUTY_RANGE);
	}
	else if(duty < 0){
		dir_state = 2; // Reverse
		pulse = -duty * (pwm_period / (float)MT_DUTY_RANGE);
	}
	else{
		dir_state = 3; // Stop (Free Running)
		pulse = 0;
	}
	
	switch(ch){
		case 0:
		MT0_OC_Struct.Pulse = pulse;
		HAL_TIM_PWM_ConfigChannel(&h_Tim2, &MT0_OC_Struct, MT0_PWM_CH);
		HAL_TIM_PWM_Start(&h_Tim2, MT0_PWM_CH);
		
		HAL_GPIO_WritePin(MT0_DIR0_GPIO, MT0_DIR0_PIN, dir_state & 0x01);
		HAL_GPIO_WritePin(MT0_DIR1_GPIO, MT0_DIR1_PIN, (dir_state >> 1) & 0x01);
		break;
		
		case 1:
		MT1_OC_Struct.Pulse = pulse;
		HAL_TIM_PWM_ConfigChannel(&h_Tim2, &MT1_OC_Struct, MT1_PWM_CH);
		HAL_TIM_PWM_Start(&h_Tim2, MT1_PWM_CH);
		
		HAL_GPIO_WritePin(MT1_DIR0_GPIO, MT1_DIR0_PIN, dir_state & 0x01);
		HAL_GPIO_WritePin(MT1_DIR1_GPIO, MT1_DIR1_PIN, (dir_state >> 1) & 0x01);
		break;
	}
} 


uint32_t MT_PWM_getDuty(uint8_t ch)
{
	switch(ch){
		case 0:
		return MT0_OC_Struct.Pulse;
		
		case 1:
		return MT1_OC_Struct.Pulse;
		
		default:
		return 0;
	}
}

void MT_Enc_init(void)
{
	/* Encoder Input */
	/* Input1 : PB4(Nucleo: D12) (TIM3_CH1) */
	/* Input2 : PB5(Nucleo: D11) (TIM3_CH2) */
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable GPIO clock */
	RCC->AHBENR |= (RCC_AHBENR_GPIOBEN);

	/* GPIO Initialize */
	GPIO_InitStruct.Pin 		= GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate 	= GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* Encoder IF Initialize */
	RCC->APB1ENR |= (RCC_APB1ENR_TIM3EN);
	h_Tim3.Instance = TIM3;
	h_Tim3.Init.Prescaler = 0;
	h_Tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	h_Tim3.Init.Period = 0xFFFF;
	h_Tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&h_Tim3);
	
	Enc_Struct.EncoderMode	= TIM_ENCODERMODE_TI12;
	Enc_Struct.IC1Polarity	= TIM_ICPOLARITY_RISING;
	Enc_Struct.IC2Polarity	= TIM_ICPOLARITY_RISING;
	Enc_Struct.IC1Selection	= TIM_ICSELECTION_DIRECTTI;
	Enc_Struct.IC2Selection	= TIM_ICSELECTION_DIRECTTI;
	Enc_Struct.IC1Prescaler	= TIM_ICPSC_DIV4;
	Enc_Struct.IC2Prescaler	= TIM_ICPSC_DIV4;
	HAL_TIM_Encoder_Init(&h_Tim3, &Enc_Struct);
	
	HAL_TIM_Encoder_Start(&h_Tim3, TIM_CHANNEL_1);
}

uint32_t MT_Enc_getValue(void)
{
	return TIM3->CNT;
}