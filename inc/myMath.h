
#ifndef __MYMATH_H
#define __MYMATH_H 0x0200

// Original Math Functions

#ifdef __cplusplus
 extern "C" {
#endif

#include <inttypes.h>
#include <Math.h>


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define GRAVITY 9.767f

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define ToRad(x) (x * DEG_TO_RAD)
#define ToDeg(x) (x * RAD_TO_DEG)

typedef union __generic_16bit{
	uint8_t b[2];
	int16_t i;
} generic_16bit;

typedef union __generic_32bit{
	uint8_t b[4];
	float f;
	int32_t i;
} generic_32bit;

typedef union __generic_64bit{
	uint8_t b[8];
	int64_t ll;
} generic_64bit;

/* Vector and Matrix Paraemters */
typedef struct __Vector3f{
	float x, y, z;
} Vector3f;

typedef struct __Vector3d{
	uint16_t x, y, z;
} Vector3d;

typedef struct __Matrix3f{
	Vector3f a;
	Vector3f b;
	Vector3f c;	
} Matrix3f;


float	Vector_Dot_Product(const Vector3f *v1, const Vector3f *v2);
void	Vector_Cross_Product(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Vector_Scale(Vector3f *out, const Vector3f *v, float scale);
void	Vector_Add(Vector3f *out, const Vector3f *v1, const Vector3f *v2);
void	Matrix_Multiply(const Matrix3f *m1, const Matrix3f *m2, Matrix3f *out);
void	Matrix_Vector_Multiply(const Matrix3f *m, const Vector3f *v, Vector3f *out);
void	Vector_Normalize(Vector3f *a);

#ifdef __cplusplus
}
#endif

#endif