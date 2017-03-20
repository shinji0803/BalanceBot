
#ifndef __AHRS_H
#define __AHRS_H 0x0200

// Attitude Heading Reference System

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_it.h"
#include "myMath.h"

void AHRS_init(void);

void AHRS_readIMU(void); // read all sensor(Gyro, Acc, Mag)

// DCM algorithm
void AHRS_dcmUpdate(float dt); // update DCM
void AHRS_dcmNormalize(void); // normalize DCM
void AHRS_driftCorrection(void); // gyro drift correction
// get AHRS gain
void AHRS_getGain(float *tempKp, float *tempKi, float *tempKp_yaw, float *tempKi_yaw);
// set AHRS gain
void AHRS_setGain(float tempKp, float tempKi, float tempKp_yaw, float tempKi_yaw);

// Gyro function
void AHRS_getGyro(Vector3f *g); // get scaled gyro value
void AHRS_getRawGyro(Vector3f *g); // get raw gyro value
void AHRS_getOmega(Vector3f *g); // Drift corrected gyro value

// Acc function
void AHRS_getAcc(Vector3f *a); // get scaled acc value
void AHRS_getRawAcc(Vector3f *a); // get raw acc value

// Mag function
void AHRS_getMag(Vector3f *m); // get scaled mag value
void AHRS_getRawMag(Vector3f *m); // get raw mag value
void AHRS_showCalibMag(void); // show calibration data

void AHRS_calcEuler(void); // calculate euler angle from DCM
void AHRS_getEuler(Vector3f *att); // get euler angle

float AHRS_magHeading(void); // get heading(rad) that corrected leaning


/* Calibration Data */
/* Mag calibration data */
static const Vector3f m_max = { 587.0f, 754.0f, 727.0f}; 
static const Vector3f m_min = { -946.0f, -686.0f, -680.0f};

/* Acc calibration data */
static const Vector3f a_max = { 32768.0f, 32768.0f, 32768.0f};
static const Vector3f a_min = { -32768.0f, -32768.0f, -32768.0f};

/* Gyro calibration data */
#define GYRO_CALIB_LOW		-40
#define GYRO_CALIB_HIGH		40

/* Sensor scale data */
#define ACCEL_X_OFFSET ((a_min.x + a_max.x) / 2.0f)
#define ACCEL_Y_OFFSET ((a_min.y + a_max.y) / 2.0f)
#define ACCEL_Z_OFFSET ((a_min.z + a_max.z) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (a_max.x - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (a_max.y - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (a_max.z - ACCEL_Z_OFFSET))


#define MAGN_X_OFFSET ((m_min.x + m_max.x) / 2.0f)
#define MAGN_Y_OFFSET ((m_min.y + m_max.y) / 2.0f)
#define MAGN_Z_OFFSET ((m_min.z + m_max.z) / 2.0f)
#define MAGN_X_SCALE (100.0f / (m_max.x - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (m_max.y - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (m_max.z - MAGN_Z_OFFSET))

#ifdef __cplusplus
}
#endif

#endif