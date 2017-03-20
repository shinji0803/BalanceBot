
#include "MPU9250.h"
#include "timer.h"

#define GYRO_SCALE	ToRad(2000.f)/ 32768.f
#define ACC_SCALE	GRAVITY / 2048.f

static I2C_HandleTypeDef h_i2c;

static void MPU9250_initI2C(void);
static uint8_t MPU9250_write(uint8_t *pData, uint16_t size);
static uint8_t MPU9250_read(uint8_t reg_add, uint8_t *r_data, uint16_t r_size);
//static uint8_t AK8963_write(uint8_t *data, uint16_t size);
//static uint8_t AK8963_read(uint8_t reg_add, uint8_t *r_data, uint16_t r_size);

void MPU9250_init(void)
{
	uint8_t temp[4];
	
	MPU9250_initI2C();
	
	// Acc and Gyro Initialize
	if(MPU9250_who() != 0x71){
		while(1);
	}

INIT_START:
	temp[0] = PWR_MGMT_1;
	temp[1] = 0x80;
	MPU9250_write(temp, 2); // reset device
	temp[0] = INT_PIN_CFG;
	temp[1] = 0x02;
	MPU9250_write(temp, 2); // enable I2C bypass to AK8963
	temp[0] = PWR_MGMT_1;
	temp[1] = 0x03;
	MPU9250_write(temp, 2); // woke up device and select z-gyro
	delay_ms(5);
	MPU9250_read(PWR_MGMT_1, temp, 1);
	if(temp[0] != 0x03) goto INIT_START; // check it has woken up
	delay_ms(10);
	
	temp[0] = PWR_MGMT_2;
	temp[1] = 0x00;
	MPU9250_write(temp, 2);
	delay_us(1000);
	temp[0] = CONFIG;
	temp[1] = 0x00;
	MPU9250_write(temp, 2); // set gyro filter to 250Hz
	temp[0] = ACCEL_CONFIG2;
	temp[1] = 0x00;
	MPU9250_write(temp, 2); // set accel filter to 460Hz
	
	temp[0] = SMPLRT_DIV;
	temp[1] = 0x00;
	MPU9250_write(temp, 2); // set sample rate to 1000Hz
	temp[0] = GYRO_CONFIG;
	temp[1] = 0x18;
	MPU9250_write(temp, 2); // set gyro scale to 2000dps
	temp[0] = ACCEL_CONFIG;
	temp[1] = 0x03 << 3;
	MPU9250_write(temp, 2); // set accel scale to 16G
	delay_us(1000);
	
	temp[0] = INT_PIN_CFG;
	temp[1] = 0x22; //
	MPU9250_write(temp, 2);	
}

static void MPU9250_initI2C(void)
{
	GPIO_InitTypeDef GPIO_initStruct;
	I2C_InitTypeDef i2c_initStruct;
	
	/* GPIO Initialize */
	RCC->AHBENR |= (RCC_AHBENR_GPIOBEN);
	/* I2C SCL:PB6 (Nucleo: A5) */
	/* I2C SDA:PB7 (Nucleo: A4) */
	GPIO_initStruct.Pin 		= GPIO_PIN_6;
	GPIO_initStruct.Mode 		= GPIO_MODE_AF_OD;
	GPIO_initStruct.Pull 		= GPIO_NOPULL;
	GPIO_initStruct.Speed 		= GPIO_SPEED_HIGH;
	GPIO_initStruct.Alternate 	= GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_initStruct);
	
	GPIO_initStruct.Pin 		= GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_initStruct);
	
	/* I2C Initialize */
	RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN);
	
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
	
	//i2c_initStruct.Timing 			= 0x10B17DB4; // SCL = 100kHz
	i2c_initStruct.Timing 			= 0x00E22163; // SCL = 400KHz
	i2c_initStruct.OwnAddress1		= 0x00;
	i2c_initStruct.AddressingMode	= I2C_ADDRESSINGMODE_7BIT;
	i2c_initStruct.DualAddressMode	= I2C_DUALADDRESS_DISABLED;
	i2c_initStruct.GeneralCallMode	= I2C_GENERALCALL_DISABLED;
	i2c_initStruct.NoStretchMode	= I2C_NOSTRETCH_DISABLED; 
	
	h_i2c.Instance = I2C1;
	h_i2c.Init = i2c_initStruct;
	HAL_I2C_Init(&h_i2c);
	/* I2C Initialize End */
}

uint8_t MPU9250_who(void)
{
	uint8_t temp;
	
	MPU9250_read(WHO_AM_I_MPU9250, &temp, 1);
	
	return temp;
}

void MPU9250_getRawGyro(int16_t *g)
{
	uint8_t temp[6];
	
	MPU9250_read( GYRO_XOUT_H, temp, 6);
	
	g[0] = ((int16_t)temp[0] << 8)| temp[1];
	g[1] = ((int16_t)temp[2] << 8)| temp[3];
	g[2] = ((int16_t)temp[4] << 8)| temp[5];	
}

void MPU9250_getRawGyroV3f(Vector3f *g)
{
	uint8_t temp[6];
	
	MPU9250_read( GYRO_XOUT_H, temp, 6);
	
	g->x = (int16_t)(((int16_t)temp[0] << 8)| temp[1]);
	g->y = (int16_t)(((int16_t)temp[2] << 8)| temp[3]);
	g->z = (int16_t)(((int16_t)temp[4] << 8)| temp[5]);
}

void MPU9250_getRawAcc(int16_t *a)
{
	uint8_t temp[6];
	
	MPU9250_read( ACCEL_XOUT_H, temp, 6);
	
	a[0] = ((int16_t)temp[0] << 8)| temp[1];
	a[1] = ((int16_t)temp[2] << 8)| temp[3];
	a[2] = ((int16_t)temp[4] << 8)| temp[5];	
}

void MPU9250_getRawAccV3f(Vector3f *a)
{
	uint8_t temp[6];
	
	MPU9250_read( ACCEL_XOUT_H, temp, 6);
	
	a->x = ((int16_t)temp[0] << 8)| temp[1];
	a->y = ((int16_t)temp[2] << 8)| temp[3];
	a->z = ((int16_t)temp[4] << 8)| temp[5];	
}

void MPU9250_getRawIMU(int16_t *a, int16_t *g, int16_t *t)
{
	uint8_t temp[14];
	
	MPU9250_read( ACCEL_XOUT_H, temp, 14);
	
	a[0] = ((int16_t)temp[0] << 8)| temp[1];
	a[1] = ((int16_t)temp[2] << 8)| temp[3];
	a[2] = ((int16_t)temp[4] << 8)| temp[5];
	
	*t = ((int16_t)temp[6] << 8)| temp[7];
	
	g[0] = ((int16_t)temp[8] << 8)| temp[9];
	g[1] = ((int16_t)temp[10] << 8)| temp[11];
	g[2] = ((int16_t)temp[12] << 8)| temp[13];
}

void MPU9250_getRawIMUV3f(Vector3f *a, Vector3f *g, int16_t *t)
{
	uint8_t temp[14];
	
	MPU9250_read( ACCEL_XOUT_H, temp, 14);
	
	a->x = (int16_t)(((int16_t)temp[0] << 8)| temp[1]);
	a->y = (int16_t)(((int16_t)temp[2] << 8)| temp[3]);
	a->z = (int16_t)(((int16_t)temp[4] << 8)| temp[5]);
	
	*t = (int16_t)(((int16_t)temp[6] << 8)| temp[7]);
	
	g->x = (int16_t)(((int16_t)temp[8] << 8)| temp[9]);
	g->y = (int16_t)(((int16_t)temp[10] << 8)| temp[11]);
	g->z = (int16_t)(((int16_t)temp[12] << 8)| temp[13]);
}

float MPU9250_getGyroScale(void)
{
	return GYRO_SCALE;
}

float MPU9250_getAccScale(void)
{
	return ACC_SCALE;
}

static uint8_t MPU9250_write(uint8_t *data, uint16_t size)
{
	//return HAL_I2C_Master_Transmit_IT(&h_i2c, MPU9250_ADD, pData, size);
	
	while(HAL_I2C_Master_Transmit(&h_i2c, MPU9250_ADD, data, size, 100) != HAL_OK);
	
	return HAL_OK;
}

static uint8_t MPU9250_read(uint8_t reg_add, uint8_t *r_data, uint16_t r_size)
{
	//return HAL_I2C_Master_Receive_IT(&h_i2c, MPU9250_ADD, pData, size);
	
	while(HAL_I2C_Master_Transmit(&h_i2c, MPU9250_ADD, &reg_add, 1, 100) != HAL_OK);
	
	while(HAL_I2C_Master_Receive(&h_i2c, MPU9250_ADD, r_data, r_size, 100) != HAL_OK);
	
	return HAL_OK;
}

/*
static uint8_t AK8963_write(uint8_t *data, uint16_t size)
{	
	while(HAL_I2C_Master_Transmit(&h_i2c, AK8963_ADD, data, size, 100) != HAL_OK);
	
	return HAL_OK;
}

static uint8_t AK8963_read(uint8_t reg_add, uint8_t *r_data, uint16_t r_size)
{
	while(HAL_I2C_Master_Transmit(&h_i2c, AK8963_ADD, &reg_add, 1, 100) != HAL_OK);
	
	while(HAL_I2C_Master_Receive(&h_i2c, AK8963_ADD, r_data, r_size, 100) != HAL_OK);
	
	return HAL_OK;
}
*/