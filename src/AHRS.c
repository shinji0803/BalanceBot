
#include "AHRS.h"

#include "hw_config.h"

#include "myMath.h"
#include "timer.h"
#include "MPU9250.h"


static Matrix3f _dcm_matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
static Vector3f _gyroOffset;

static Vector3f _rawG = { 0.0f, 0.0f, 0.0f}, _rawA = { 0.0f, 0.0f, 0.0f}, _rawM = { 0.0f, 0.0f, 0.0f};
static Vector3f _scaledG = { 0.0f, 0.0f, 0.0f}, _scaledA = { 0.0f, 0.0f, 0.0f}, _scaledM = { 0.0f, 0.0f, 0.0f};
static float _imu_temp = 0.0f;
static float _mag_heading = 0.0f;

static float _gyroScale, _accScale;

static float _roll, _pitch, _yaw;

//DCM Parameter
static Vector3f _omega = { 0.0f, 0.0f, 0.0f};
static Vector3f _omega_vector = { 0.0f, 0.0f, 0.0f};
static Vector3f _omega_P = { 0.0f, 0.0f, 0.0f};
static Vector3f _omega_I = { 0.0f, 0.0f, 0.0f};
static Vector3f _errorRollPitch = { 0.0f, 0.0f, 0.0f};
static Vector3f _errorYaw = { 0.0f, 0.0f, 0.0f};

//AHRS Gains
static float _Kp = 0.6f, _Kp_yaw = 3.0f, _Ki = 0.0008f, _Ki_yaw = 0.001f;

static void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll);
static uint8_t check_gyro_calib_data(Vector3f *g);

void AHRS_init(void)
{
	Vector3f temp = { 0.0f, 0.0f, 0.0f};
	Vector3f temp1, temp2;
	Vector3f sum = { 0.0f, 0.0f, 0.0f};
	
	const Vector3f xAxis = {1.0f, 0.0f, 0.0f};
	
	int32_t sum_count = 0;
	
	MPU9250_init();
	_gyroScale = MPU9250_getGyroScale();
	_accScale = MPU9250_getAccScale();
	
	delay_ms(100);
	
	/* Gyro scale calibration */
	cputs("Gyro Calibration");
	
	while(1){
		sum_count = 0;
		while(1){
			MPU9250_getRawGyroV3f(&temp);
			
			if(check_gyro_calib_data(&temp) != 0) break;
			sum.x += temp.x;
			sum.y += temp.y;
			sum.z += temp.z;
			sum_count ++;
			
			cputs(".");
			delay_us(200000);
			if(sum_count >= 10) break;
		}
		
		if(sum_count >= 10) break;
		else{
			printf("\r\nDo not move IMU.\r\nRecalibrate");
			sum.x = sum.y = sum.z = 0.0f;
			sum_count = 0;
			delay_ms(500);
		}
		
	}
	printf("\r\n");
	_gyroOffset.x = sum.x / 10.0f; 
	_gyroOffset.y = sum.y / 10.0f; 
	_gyroOffset.z = sum.z / 10.0f;
	
	printf("Gyro Offset ( %f, %f, %f)\r\n", _gyroOffset.x, _gyroOffset.y, _gyroOffset.z);
	
	//Initialize DCM Matrix
	Vector3f att;
	//Get Roll and Pitch
	AHRS_readIMU();
	AHRS_getAcc(&temp);
	att.y = -atan2(temp.x, sqrt(temp.y * temp.y + temp.z * temp.z));
	Vector_Cross_Product(&temp1, &temp, &xAxis);
	Vector_Cross_Product(&temp2, &xAxis, &temp1);
	att.x = -atan2( temp2.y, temp2.z);
	//Get Yaw
	//att.z = AHRS_mag_heading();
	att.z = 0;
	printf("Att ( %f, %f, %f)\r\n", att.x, att.y, att.z);
	init_rotation_matrix(&_dcm_matrix, att.z, att.y, att.x);
}

static uint8_t check_gyro_calib_data(Vector3f *g)
{
	uint8_t error_count = 0;
	if(g->x > GYRO_CALIB_HIGH || g->x < GYRO_CALIB_LOW) error_count ++;
	if(g->y > GYRO_CALIB_HIGH || g->y < GYRO_CALIB_LOW) error_count ++;
	if(g->z > GYRO_CALIB_HIGH || g->z < GYRO_CALIB_LOW) error_count ++;
	return error_count;
}

void AHRS_readIMU(void)
{
	Vector3f tempA, tempG;
	int16_t temp;
	
	/* Read Gyro, Acc and Temp*/
	MPU9250_getRawIMUV3f(&tempA, &tempG, &temp);
	_imu_temp = (float)temp / 333.87f + 21.f;
	
	/* Gyro Scaling */
	
	_rawG.x = -(tempG.y - _gyroOffset.y);
	_rawG.y = -(tempG.x - _gyroOffset.x);
	_rawG.z = tempG.z - _gyroOffset.z;
	/*
	_rawG.x = (tempG.z - _gyroOffset.z);
	_rawG.y = -(tempG.x - _gyroOffset.x);
	_rawG.z = -(tempG.y - _gyroOffset.y);	
	*/
	Vector_Scale(&_scaledG, &_rawG, _gyroScale);
	
	/* Acc read */
	
	_rawA.x = -tempA.y;
	_rawA.y = -tempA.x;
	_rawA.z = tempA.z;
	/*
	_rawA.x = tempA.z;
	_rawA.y = -tempA.x;
	_rawA.z = -tempA.y;
	*/
	Vector_Scale(&_scaledA, &_rawA, _accScale);
	
	/* Mag read */
}

void AHRS_dcmUpdate(float dt)
{
	Matrix3f dcm_update_matrix;
	Matrix3f temp_matrix = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	
	Vector_Add(&_omega, &_scaledG, &_omega_I);
	Vector_Add(&_omega_vector, &_omega, &_omega_P);
	
	dcm_update_matrix.a.x = 0;
	dcm_update_matrix.a.y = -dt * _omega_vector.z;
	dcm_update_matrix.a.z = dt * _omega_vector.y;
	dcm_update_matrix.b.x = dt * _omega_vector.z;
	dcm_update_matrix.b.y = 0;
	dcm_update_matrix.b.z = -dt * _omega_vector.x;
	dcm_update_matrix.c.x = -dt * _omega_vector.y;
	dcm_update_matrix.c.y = dt * _omega_vector.x;
	dcm_update_matrix.c.z = 0;
	
	Matrix_Multiply(&_dcm_matrix, &dcm_update_matrix, &temp_matrix);
	
	_dcm_matrix.a.x += temp_matrix.a.x;
	_dcm_matrix.a.y += temp_matrix.a.y;
	_dcm_matrix.a.z += temp_matrix.a.z;
	_dcm_matrix.b.x += temp_matrix.b.x;
	_dcm_matrix.b.y += temp_matrix.b.y;
	_dcm_matrix.b.z += temp_matrix.b.z;
	_dcm_matrix.c.x += temp_matrix.c.x;
	_dcm_matrix.c.y += temp_matrix.c.y;
	_dcm_matrix.c.z += temp_matrix.c.z;
}

void AHRS_dcmNormalize(void)
{
	float error = 0.0f;
	Matrix3f temp;
	float renorm = 0.0f;
	
	error = -Vector_Dot_Product(&_dcm_matrix.a, &_dcm_matrix.b) * 0.5f;
	
	Vector_Scale(&temp.a, &_dcm_matrix.b, error);
	Vector_Scale(&temp.b, &_dcm_matrix.a, error);
	
	Vector_Add(&temp.a, &temp.a, &_dcm_matrix.a);
	Vector_Add(&temp.b, &temp.b, &_dcm_matrix.b);
	
	Vector_Cross_Product(&temp.c, &temp.a, &temp.b);
	
	renorm= 0.5f * (3 - Vector_Dot_Product(&temp.a, &temp.a));
	Vector_Scale(&_dcm_matrix.a, &temp.a, renorm);
	
	renorm= 0.5f * (3 - Vector_Dot_Product(&temp.b, &temp.b));
	Vector_Scale(&_dcm_matrix.b, &temp.b, renorm);
	
	renorm= 0.5f * (3 - Vector_Dot_Product(&temp.c, &temp.c));
	Vector_Scale(&_dcm_matrix.c, &temp.c, renorm);
}

void AHRS_driftCorrection(void)
{
	//float mag_heading_x;
	//float mag_heading_y;
	float errorCourse;

	Vector3f scaled_omega_P = {0.0f, 0.0f, 0.0f};
	Vector3f scaled_omega_I = {0.0f, 0.0f, 0.0f};
	float accel_magnitude;
	float accel_weight;
	
	/* calculate heading from mag */
	//_mag_heading = AHRS_magHeading();
	_mag_heading = 0.f;
	
	/* Roll and Pitch */
	accel_magnitude = sqrtf(_scaledA.x * _scaledA.x + _scaledA.y * _scaledA.y + _scaledA.z * _scaledA.z);
	accel_magnitude = accel_magnitude / GRAVITY;
	accel_weight = constrain(1.0f - 2.0f * fabsf(1.0f - accel_magnitude), 0, 1.0f);
	
	Vector_Cross_Product(&_errorRollPitch, &_scaledA, &_dcm_matrix.c);
	Vector_Scale(&_omega_P, &_errorRollPitch, _Kp * accel_weight);
	
	Vector_Scale(&scaled_omega_I, &_errorRollPitch, _Ki * accel_weight);
	Vector_Add(&_omega_I, &_omega_I, &scaled_omega_I);
	
	/* YAW */
	/*
	mag_heading_x = cosf(_mag_heading);
	mag_heading_y = sinf(_mag_heading);
	errorCourse = (_dcm_matrix.a.x * mag_heading_y) - (_dcm_matrix.b.x * mag_heading_x);
	*/
	errorCourse = 0.f;
	Vector_Scale(&_errorYaw, &_dcm_matrix.c, errorCourse);
  
	Vector_Scale(&scaled_omega_P, &_errorYaw, _Kp_yaw);
	Vector_Add(&_omega_P, &_omega_P, &scaled_omega_P);
  
	Vector_Scale(&scaled_omega_I, &_errorYaw, _Ki_yaw);
	Vector_Add(&_omega_I, &_omega_I, &scaled_omega_I);
}

void AHRS_calcEuler()
{
	_pitch = -asin(_dcm_matrix.c.x);
	_roll = atan2(_dcm_matrix.c.y, _dcm_matrix.c.z);
	_yaw = atan2(_dcm_matrix.b.x, _dcm_matrix.a.x);
}

void AHRS_getGain(float *tempKp, float *tempKi, float *tempKp_yaw, float *tempKi_yaw)
{
	*tempKp = _Kp;
	*tempKi = _Ki;
	*tempKp_yaw = _Kp_yaw;
	*tempKi_yaw = _Ki_yaw;
}

void AHRS_setGain(float tempKp, float tempKi, float tempKp_yaw, float tempKi_yaw)
{
	_Kp = tempKp;
	_Ki = tempKi;
	_Kp_yaw = tempKp_yaw;
	_Ki_yaw = tempKi_yaw;
}

void AHRS_getGyro(Vector3f *g)
{
	g->x = _scaledG.x;
	g->y = _scaledG.y;
	g->z = _scaledG.z;
}

void AHRS_getRawGyro(Vector3f *g)
{
	g->x = _rawG.x;
	g->y = _rawG.y;
	g->z = _rawG.z;
}

void AHRS_getOmega(Vector3f *g)
{
	g->x = _omega_vector.x;
	g->y = _omega_vector.y;
	g->z = _omega_vector.z;
}

void AHRS_getAcc(Vector3f *a)
{
	a->x = _scaledA.x;
	a->y = _scaledA.y;
	a->z = _scaledA.z;
}

void AHRS_getRawAcc(Vector3f *a)
{
	a->x = _rawA.x;
	a->y = _rawA.y;
	a->z = _rawA.z;
}

void AHRS_getMag(Vector3f *m)
{
	m->x = (_rawM.x - MAGN_X_OFFSET) * MAGN_X_SCALE;
	m->y = (_rawM.y - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
	m->z = (_rawM.z - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
}

void AHRS_getRawMag(Vector3f *m)
{
	m->x = _rawM.x;
	m->y = _rawM.y;
	m->z = _rawM.z;
}

void AHRS_show_calib_mag(void)
{
	static Vector3f temp_max, temp_min;
	static uint32_t last_call = 0;
	
	if(millis() - last_call > 200){
		memset(&temp_max, 0, sizeof(temp_max));
		memset(&temp_min, 0, sizeof(temp_min));
	}
	last_call = millis();
	
	if(_rawM.x > temp_max.x) temp_max.x = _rawM.x;
	if(_rawM.x < temp_min.x) temp_min.x = _rawM.x;
	
	if(_rawM.y > temp_max.y) temp_max.y = _rawM.y;
	if(_rawM.y < temp_min.y) temp_min.y = _rawM.y;
	
	if(_rawM.z > temp_max.z) temp_max.z = _rawM.z;
	if(_rawM.z < temp_min.z) temp_min.z = _rawM.z;
	
	printf("> M: X(%5.0f, %5.0f) Y(%5.0f, %5.0f), Z(%5.0f, %5.0f)\r\n",
				temp_max.x, temp_min.x, temp_max.y, temp_min.y, temp_max.z, temp_min.z);
}

static void init_rotation_matrix(Matrix3f *m, float yaw, float pitch, float roll)
{	
	float c1 = cosf(roll);
	float s1 = sinf(roll);
	float c2 = cosf(pitch);
	float s2 = sinf(pitch);
	float c3 = cosf(yaw);
	float s3 = sinf(yaw);

	// Euler angles, right-handed, intrinsic, XYZ convention
	// (which means: rotate around body axes Z, Y', X'') 
	m->a.x = c2 * c3;
	m->a.y = c3 * s1 * s2 - c1 * s3;
	m->a.z = s1 * s3 + c1 * c3 * s2;
	
	m->b.x = c2 * s3;
	m->b.y = c1 * c3 + s1 * s2 * s3;
	m->b.z = c1 * s2 * s3 - c3 * s1;
	
	m->c.x = -s2;
	m->c.y = c2 * s1;
	m->c.z = c1 * c2;
}

void AHRS_getEuler(Vector3f *att)
{
	att->x = _roll;
	att->y = _pitch;
	att->z = _yaw;
}

float AHRS_magHeading(void)
{
	float mag_x;
	float mag_y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;
	
	AHRS_getMag(&_scaledM);
	
	cos_roll = cosf(_roll);
	sin_roll = sinf(_roll);
	cos_pitch = cosf(_pitch);
	sin_pitch = sinf(_pitch);
  
	// Tilt compensated magnetic field X
	mag_x = _scaledM.x * cos_pitch + _scaledM.y * sin_roll * sin_pitch + _scaledM.z * cos_roll * sin_pitch;
	// Tilt compensated magnetic field Y
	mag_y = _scaledM.y * cos_roll - _scaledM.z * sin_roll;
  
	return atan2(-mag_y, mag_x);
}
	
	
