//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float mq0, mq1, mq2, mq3;
extern volatile float mq0q0, mq0q1, mq0q2, mq0q3, mq1q1, mq1q2, mq1q3, mq2q2, mq2q3, mq3q3;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float dt);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
