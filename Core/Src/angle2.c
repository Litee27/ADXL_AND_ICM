#include "angle2.h"
#include "filter.h"
#define micros 1000*HAL_GetTick 
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f,
             q3 = 0.0f;    /** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f,
             dq3 = 0.0f;    /** quaternion of sensor frame relative to auxiliary frame */
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static float halfT=0.000139f;
//static float k10=0.0f,k11=0.0f,k12=0.0f,k13=0.0f;
//static float k20=0.0f,k21=0.0f,k22=0.0f,k23=0.0f;
//static float k30=0.0f,k31=0.0f,k32=0.0f,k33=0.0f;

imu_t imu = {0};
int califlag=0;


void ICM_GET_ANGLE(void)
{
    //! Time constant
    float dt = 0.001 * Math_PERIOD;     //s
    static uint32_t tPrev = 0, startTime = 0; //us
    uint32_t now;
//    uint8_t i;
    
    /* output euler angles */
    float euler[3] = {0.0f, 0.0f, 0.0f};    //rad

    /* Initialization */
    float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };       /**< init: identity matrix */
    float acc[3] = {0.0f, 0.0f, 0.0f};      //m/s^2
    float gyro[3] = {0.0f, 0.0f, 0.0f};     //rad/s

    //need to calc gyro offset before imu start working
    static float gyro_offsets_sum[3] = { 0.0f, 0.0f, 0.0f }; // gyro_offsets[3] = { 0.0f, 0.0f, 0.0f },
//    static float acc_offsets_sum[3] = { 0.0f, 0.0f, 0.0f };
		static float gyro_adc_offsets_sum[3] = { 0.0f, 0.0f, 0.0f };
//		static float a;
    static uint16_t offset_count = 0;
    now = micros();
    dt = (tPrev > 0) ? (now - tPrev) / 1000000.0f : 0;
    tPrev = now;
    if(dt==0)return;
    ReadIMUSensorHandle();
    imu.accb[0]=imu.accg[0];
    imu.accb[1]=imu.accg[1];
    imu.accb[2]=imu.accg[2];
    
		//如果输入cali等于1，那么开始校准，可以开机给初始值1
		if(imu.caliGyro){
			imu.ready = 0;
			
			if (startTime == 0)
         startTime = now;

			gyro_offsets_sum[0] += imu.gyroRaw[0];
			gyro_offsets_sum[1] += imu.gyroRaw[1];
			gyro_offsets_sum[2] += imu.gyroRaw[2];
			gyro_adc_offsets_sum[0] += imu.gyroADC[0];
			gyro_adc_offsets_sum[1] += imu.gyroADC[1];
			gyro_adc_offsets_sum[2] += imu.gyroADC[2];
//            acc_offsets_sum[0] += imu.accRaw[0];
//            acc_offsets_sum[1]  +=  imu.accRaw[1];
//            acc_offsets_sum[2]  +=   imu.accRaw[2];
			offset_count++;
			
			if (now > startTime + GYRO_CALC_TIME)
        {
//            imu.gyroOffset[0] = gyro_offsets_sum[0] / offset_count;
//            imu.gyroOffset[1] = gyro_offsets_sum[1] / offset_count;
//            imu.gyroOffset[2] = gyro_offsets_sum[2] / offset_count;
//							imu.gyroADCOffset[0] = gyro_adc_offsets_sum[0] / offset_count;
//							imu.gyroADCOffset[1] = gyro_adc_offsets_sum[1] /offset_count;
						for(uint8_t i = 0; i < 3; ++i)
						{
//							imu.gyroOffset[i] = gyro_offsets_sum[i] / offset_count;
							imu.gyroADCOffset[i] = gyro_adc_offsets_sum[i] / offset_count;
						}
						
//            acc_offsets_sum[0] = acc_offsets_sum[0] / offset_count;
//            acc_offsets_sum[1] = acc_offsets_sum[1] / offset_count;
//            acc_offsets_sum[2] = acc_offsets_sum[2] / offset_count;
//						imu.accOffset[0] = acc_offsets_sum[0] / offset_count;
//						imu.accOffset[1] = acc_offsets_sum[1] / offset_count;
//						imu.accOffset[2] = acc_offsets_sum[2] / offset_count;
//                   acc_offsets_sum
//            imu.gyroOffset[0] = -6.98f* GYRO_SCALE * PI / 180.f;
//            imu.gyroOffset[1] = -9.f* GYRO_SCALE * PI / 180.f;
//            imu.gyroOffset[2] = -13.7f* GYRO_SCALE * PI / 180.f;
//            AccVector  =   sqrt(acc_offsets_sum[0]  *acc_offsets_sum[0]+
//            acc_offsets_sum[1]  *acc_offsets_sum[1]+
//            acc_offsets_sum[2]  *acc_offsets_sum[2]);
            offset_count = 0;
						for(uint8_t i = 0; i < 3; ++i)
						{
							gyro_offsets_sum[i] = 0;
							gyro_adc_offsets_sum[i] = 0;
						}
//            gyro_offsets_sum[0] = 0;
//            gyro_offsets_sum[1] = 0;
//            gyro_offsets_sum[2] = 0;
//            acc_offsets_sum[0] = 0;								//accoffset给0比较稳妥，给0的话就特别稳。不需要offset
//            acc_offsets_sum[1]  =  0  ;
//            acc_offsets_sum[2]  =   0  ;
            q0 = 1.0f, q1 = 0.0f, q2 = 0.0f,
            q3 = 0.0f;    /** quaternion of sensor frame relative to auxiliary frame */
            dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f,
            dq3 = 0.0f;    /** quaternion of sensor frame relative to auxiliary frame */
            q0q0 = 0, q0q1 = 0, q0q2 = 0, q0q3 = 0;
            q1q1 = 0, q1q2 = 0, q1q3 = 0;
            q2q2 = 0, q2q3 = 0;
            q3q3 = 0;
            
            imu.ready = 1;
            startTime = 0;
						imu.caliGyro = 0;
						//校准完成
///////////////////////////////////////////////////////此处可能可以使用flash来保存
//						HAL_NVIC_SystemReset();
        
				}			//end of >calitime
			return;
			
		} 			//end of Calibrate
		


    gyro[0] = imu.gyro[0] - imu.gyroOffset[0];
    gyro[1] = imu.gyro[1] - imu.gyroOffset[1];
    gyro[2] = imu.gyro[2] - imu.gyroOffset[2];

    acc[0] = imu.accb[0];
    acc[1] = imu.accb[1];
    acc[2] = imu.accb[2];
/*change*/ 
		

	
		//Test_Send_RawData((short)(acc[0]*100), (short)(acc[1]*100), (short)(acc[2]*100),(short)(gyro[0]*100), (short)(gyro[1]*100), (short)(gyro[2]*100),(short)(mag[0]), (short)(mag[1]), (short)(mag[2]));
//		mag[0] = correctionMat[0][0]*mag[0] + correctionMat[0][1]*mag[1] + correctionMat[0][2]*mag[2];
//		mag[1] = correctionMat[1][0]*mag[0] + correctionMat[1][1]*mag[1] + correctionMat[1][2]*mag[2];
//		mag[2] = correctionMat[2][0]*mag[0] + correctionMat[2][1]*mag[1] + correctionMat[2][2]*mag[2];
		
		
//    mag[0]=MagData.MagRaw[0];
//    mag[1]=MagData.MagRaw[1];
//    mag[2]=MagData.MagRaw[2];
    // NOTE : Accelerometer is reversed.
    // Because proper mount of PX4 will give you a reversed accelerometer readings.

/*change*/
//    NonlinearSO3AHRSupdate(gyro[0], gyro[1], gyro[2],
//                           acc[0], acc[1], acc[2],
//                           mag[0], mag[1], mag[2],
//                           so3_comp_params_Kp,
//                           so3_comp_params_Ki,
//                           dt);
//		MadgwickAHRSupdate(gyro[0], -gyro[1], -gyro[2],acc[0], acc[1], acc[2],mag[1], mag[0], mag[2],dt); 


//			MadgwickAHRSupdate(0,0,0,0, 0, 1,mag[1], mag[0], mag[2],dt);   //ok
//		MadgwickAHRSupdate(0,0,0,-acc[1], -acc[0], acc[2],0,0,0,dt); 			//ok
//			MadgwickAHRSupdate(gyro[1],gyro[0], -gyro[2],0, 0, 1,0,0,0,dt); 	//ok

		if(califlag==0)
		{
			MadgwickAHRSupdate(gyro[1],gyro[0], -gyro[2],-acc[1], -acc[0], acc[2],0.0f,0.0f,0.0f,dt);
		}
		else
		{
			printf("NULL\n");
//			MadgwickAHRSupdate(gyro[1],gyro[0], -gyro[2],-acc[1], -acc[0], acc[2],mag[1], mag[0], mag[2],dt);
		}
				
//		Test_Send_RawData(1,2,3,4,5,6,7,8,9);
    // Convert q->R, This R converts inertial frame to body frame.
//    Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
//    Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3); // 12
//    Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2); // 13
//    Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3); // 21
//    Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
//    Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1); // 23
//    Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2); // 31
//    Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1); // 32
//    Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
		/*change*/
    Rot_matrix[0] = mq0q0 + mq1q1 - mq2q2 - mq3q3;// 11
    Rot_matrix[1] = 2.f * (mq1 * mq2 + mq0 * mq3); // 12
    Rot_matrix[2] = 2.f * (mq1 * mq3 - mq0 * mq2); // 13
    Rot_matrix[3] = 2.f * (mq1 * mq2 - mq0 * mq3); // 21
    Rot_matrix[4] = mq0q0 - mq1q1 + mq2q2 - mq3q3;// 22
    Rot_matrix[5] = 2.f * (mq2 * mq3 + mq0 * mq1); // 23
    Rot_matrix[6] = 2.f * (mq1 * mq3 + mq0 * mq2); // 31
    Rot_matrix[7] = 2.f * (mq2 * mq3 - mq0 * mq1); // 32
    Rot_matrix[8] = mq0q0 - mq1q1 - mq2q2 + mq3q3;// 33

    //1-2-3 Representation.
    //Equation (290)
    //Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
    // Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);    //! Roll
    euler[1] = -asinf(Rot_matrix[2]);                   //! Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);

    
//    //DCM . ground to body
//    for (i = 0; i < 9; i++)
//    {
//        *(&(imu.DCMgb[0][0]) + i) = Rot_matrix[i];
//    }
    
    imu.rollRad = euler[0];
    imu.pitchRad = euler[1];
    imu.yawRad = -euler[2];
/*change*/
//    imu.roll = euler[0] * 180.0f / PI;
//    imu.pitch = euler[1] * 180.0f / PI;
//    imu.yaw = -euler[2] * 180.0f / PI;

		imu.roll  = atan2f(mq0*mq1 + mq2*mq3, 0.5f - mq1*mq1 - mq2*mq2)* 180.0f / PI;
		imu.pitch = asinf(-2.0f * (mq1*mq3 - mq0*mq2))* 180.0f / PI;
		imu.yaw   = atan2f(mq1*mq2 + mq0*mq3, 0.5f - mq2*mq2 - mq3*mq3)* 180.0f / PI;
}
void ReadIMUSensorHandle(void)
{
    uint8_t i;
ICM_DATA g_Raw_Data;
	ICM_Read_Data(&g_Raw_Data);
    imu.accADC[0] = g_Raw_Data.acc_x;
    imu.accADC[1] = g_Raw_Data.acc_y;
    imu.accADC[2] = g_Raw_Data.acc_z;
    imu.gyroADC[0] = g_Raw_Data.gyro_x - imu.gyroADCOffset[0];
    imu.gyroADC[1] = g_Raw_Data.gyro_y - imu.gyroADCOffset[1];
    imu.gyroADC[2] = g_Raw_Data.gyro_z - imu.gyroADCOffset[2];

		//归一化
    for (i = 0; i < 3; i++)
    {
        imu.accRaw[i] = (float)imu.accADC[i] * ACC_SCALE * CONSTANTS_ONE_G ;	
									//m/s^2
        imu.gyroRaw[i] = (float)imu.gyroADC[i] * GYRO_SCALE * PI / 180.f;
									//deg/s
    }
		imu.magRaw[0]  = imu.magADC[0];
		imu.magRaw[1]  = imu.magADC[1];
		imu.magRaw[2]  = imu.magADC[2];

    imu.accg[0] = LPF2pApply_1(imu.accRaw[0] - imu.accOffset[0]);
    imu.accg[1] = LPF2pApply_2(imu.accRaw[1] - imu.accOffset[1]);
    imu.accg[2] = LPF2pApply_3(imu.accRaw[2] - imu.accOffset[2]);
//    imu.accg[0] = imu.accRaw[0] *0.02+imu.accg[0] *0.98f;
//    imu.accg[1] = imu.accRaw[1] *0.02+imu.accg[1] *0.98f;
//    imu.accg[2] = imu.accRaw[2] *0.02+imu.accg[2] *0.98f;
    imu.gyro[0] = LPF2pApply_4(imu.gyroRaw[0]);
    imu.gyro[1] = LPF2pApply_5(imu.gyroRaw[1]);
    imu.gyro[2] = LPF2pApply_6(imu.gyroRaw[2]);

		
}
float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}
