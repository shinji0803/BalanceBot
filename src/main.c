
#include "hw_config.h"

float p_gain = 1.4, i_gain = 80.0, d_gain = 0.02;
float target = 1.0f;
Vector3f att, omega;

float out, error, error_rate;
float last_error = 0.0f, error_sum = 0.0f;

int16_t duty, duty0, duty1;
int32_t sum_duty;

void ahrs_update(void);
void motor_control(void);
void check_Input(void);
void ahrs_gainAdjust(void);

int main(void)
{
	uint32_t now;
	uint32_t timer[5] = { 0, 0, 0, 0, 0};
	
	uint32_t start = 0, end = 0;
	
	Set_System();
	init_SysTick();
	
	conio_init( 2, 230400UL);

	AHRS_init();
	//MPU9250_init();
	
	MT_PWM_init(20000UL);
	MT_PWM_setDuty(0, 0);
	MT_PWM_setDuty(1, 0);
	
	while(!keypressed());
	
	for(;;){
		now = millis();
		
		if(now - timer[0] >= 1000){
			timer[0] = now;
			LED_D3_TOGGLE();
		}
		
		if(now - timer[1] >= 100){
			timer[1] = now;
			
		}
		
		if(now - timer[2] >= 20){
			timer[2] = now;
			
			check_Input();
			
			
		}
		
		if(now - timer[3] >= 10){
			timer[3] = now;
			
			motor_control();
		}
		
		if(now - timer[4] >= 5){
			timer[4] = now;
			
			start = micros();
			ahrs_update();
			AHRS_getEuler(&att);
			AHRS_getOmega(&omega);
			//AHRS_getRawGyro(&omega);
			
			att.y = ToDeg(att.x);
			omega.y = ToDeg(omega.x);
			end = micros();
		}
	}
}

void ahrs_update(void)
{
	uint32_t now, dt;
	static uint32_t last_update = 0;
	
	/* Update AHRS */
	now = micros();
	dt = now - last_update;
	
	if(dt > 200000){
		last_update = now;
		return;
	}
	AHRS_readIMU();
	last_update = now;
	
	if(dt == 0) dt = 1000;
	AHRS_dcmUpdate((float)dt / 1000000.f);
	
	AHRS_dcmNormalize();
	AHRS_driftCorrection();
	AHRS_calcEuler();
}

#define DEAD_BAND 1500
#define ERROR_LIMIT 20.0f
void motor_control(void)
{	
	float rate_target;
	rate_target = (target - att.y);
	
	if((rate_target > ERROR_LIMIT) || (rate_target < -ERROR_LIMIT)){
		MT_PWM_setDuty(0, 0);
		MT_PWM_setDuty(1, 0);
		duty = 0;
		last_error = 0;
		return;
	}
	
	rate_target *= i_gain;
	error = rate_target - omega.y;	
	error_rate = (error - last_error) / 0.01f;
	last_error = error;
	
	out = p_gain * error - d_gain * error_rate;
	duty = (int16_t)out;
	
	if(duty < 0){
		duty0 = duty - DEAD_BAND;
		duty1 = duty - DEAD_BAND;
	}
	if(duty > 0){
		duty0 = duty + DEAD_BAND;
		duty1 = duty + DEAD_BAND;
	}
	
	MT_PWM_setDuty(0, duty0);
	MT_PWM_setDuty(1, -duty1);	
}

void check_Input(void)
{
	printf("%f\t%f\t%f\t%f\t%d\t%f\t%f\n", 
					p_gain, d_gain, i_gain, target, duty, error, error_rate);
	
	if(keypressed() != 1) return;
	
	switch(getch()){
		case 'q':
		p_gain += 0.05f;
		break;
		
		case 'a':
		p_gain -= 0.05f;
		break;
		
		case 'w':
		d_gain += 0.0001f;
		break;
		
		case 's':
		d_gain -= 0.0001f;
		break;
		
		case 'e':
		i_gain += 0.5f;
		break;
		
		case 'd':
		i_gain -= 0.5f;
		break;
		
		case '+':
		target += 0.1f;
		break;
		
		case '-':
		target -= 0.1f;
		break;
		
		default:
		return;
	}
}

void ahrs_gainAdjust(void)
{
	float g1, g2, g3, g4;
	AHRS_getGain(&g1, &g2, &g3, &g4);
	
	printf("%f\t%f\t%f\t%f\t%f\t%f\n", 
					g1, g2, g3, g4, att.y, omega.y);
	
	switch(getch()){
		case 'q':
		g1 += 0.1f;
		break;
		
		case 'a':
		g1 -= 0.1f;
		break;
		
		case 'w':
		g2 += 0.0001f;
		break;
		
		case 's':
		g2 -= 0.0001f;
		break;
		
		case 'e':
		g3 += 0.1f;
		break;
		
		case 'd':
		g3 -= 0.1f;
		break;
		
		case '+':
		g4 += 0.001f;
		break;
		
		case '-':
		g4 -= 0.001f;
		break;
		
		default:
		return;
	}
	AHRS_setGain(g1, g2, g3, g4);
}