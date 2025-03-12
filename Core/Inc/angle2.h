#include <stdint.h>
#include "MadgwickAHRS.h"
#include <stdio.h>
#include "ICM.h"
#define Math_PERIOD (10)
#define Kp 0.49f     //Kp比例增益 决定了加速度计和磁力计的收敛速度                  
// integral gain governs rate of convergenceof gyroscope biases
#define Ki 0.000008f   //Ki积分增益 决定了陀螺仪偏差的收敛速度  
#define ACC_CALC_TIME  1000//ms
#define GYRO_CALC_TIME   1000000l	//us
#define 	PI 		3.1415926535f
#define ACC_SCALE  (SENSOR_MAX_G/32768.0f)
#define GYRO_SCALE  (SENSOR_MAX_W/32768.0f)
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define SENSOR_MAX_G 8.0f       //constant g        // tobe fixed to 8g. but IMU need to  correct at the same time
#define SENSOR_MAX_W 2000.0f    //deg/s
//float init_ax, init_ay, init_az, init_gx, init_gy, init_gz;


typedef struct IMU_tt
{
uint8_t caliPass;
uint8_t caliGyro;
uint8_t ready;
int16_t accADC[3];
int16_t gyroADC[3];
int16_t magADC[3];
float 	accRaw[3];		//m/s^2
float 	gyroRaw[3];		//rad/s 
float 	magRaw[3];		//
float   accOffset[3];		//m/s^2
float   gyroOffset[3]; 
float 	gyroADCOffset[3];
float   accb[3];		//filted, in body frame
float   accg[3];
float   gyro[3];
float   DCMgb[3][3];
float   q[4];
float   roll;				//deg
float   pitch;
float 	yaw;
float   rollRad;				//rad
float   pitchRad;
float 	yawRad;
}imu_t;
extern imu_t imu;
void ReadIMUSensorHandle(void);
float invSqrt(float number);
void ICM_GET_ANGLE(void);
