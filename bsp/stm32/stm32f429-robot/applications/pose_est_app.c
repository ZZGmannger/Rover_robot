#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include "pose_est_app.h"
#include "math.h"
#include "mpu6xxx.h"


#define DBG_TAG "pose"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


typedef struct kalman
{
	/* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
}kalman_t;

void kalman_init(kalman_t* kf)
{
	kf->Q_angle =0.001f;
	kf->Q_bias = 0.003f;
	kf->R_measure = 0.03f;
	
	kf->angle = 0.0f;
	kf->bias = 0.0f;
	
	kf->P[0][0] = 0.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_get_angle(kalman_t* kf ,float newAngle, float newRate, float dt)
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
	double rate = newRate - kf->bias;
	kf->angle += dt*  rate;
	
	
	// Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += dt * kf->Q_bias ;
	
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = kf->P[0][0] + kf->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - kf->angle; // Angle difference
    /* Step 6 */
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_tmp = kf->P[0][0];
    float P01_tmp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_tmp;
    kf->P[0][1] -= K[0] * P01_tmp;
    kf->P[1][0] -= K[1] * P00_tmp;
    kf->P[1][1] -= K[1] * P01_tmp;

    return kf->angle;
}
 
// Used to set angle, this should be set as the starting angle
void kalman_set_angle(kalman_t* kf  ,float angle) 
{
	kf->angle = angle; 
}

// Return the unbiased rate
float kalman_get_rate(kalman_t* kf) 
{
	return kf->rate; 
}

/* These are used to tune the Kalman filter */
void kalman_set_Qangle(kalman_t* kf , float Q_angle) 
{ 
	kf->Q_angle = Q_angle;
}
void kalman_setQbias(kalman_t* kf,float Q_bias)
{
	kf->Q_bias = Q_bias; 
}
void kalman_set_Rmeasure(kalman_t* kf,float R_measure) 
{
	kf->R_measure = R_measure;
}

float kalman_get_Qangle(kalman_t* kf) 
{
	return kf->Q_angle; 
}
float kalman_get_Qbias(kalman_t* kf) 
{ 
	return kf->Q_bias; 
}

float kalman_get_Rmeasure(kalman_t* kf) 
{
	return kf->R_measure; 
}
/*=====================================================================================================*/


#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

kalman_t kalman_x; // Create the Kalman instances
kalman_t kalman_y;

rt_thread_t pose_thread;

/* IMU Data */
double acc_x, acc_y, acc_z;
double gyro_x, gyro_y, gyro_z;
int16_t temp_raw;

double gyro_x_angle, gyro_y_angle; // Angle calculate using the gyro only
double comp_angle_x, comp_angle_y; // Calculated angle using a complementary filter
double kal_angle_x, kal_angle_y; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

struct mpu6xxx_device * mpu6050;
struct mpu6xxx_3axes accel, gyro;
float temp;


//#define RAD_TO_DEG (360/PI/2) // 弧度转角度的转换率
//#define DEG_TO_RAD (2*PI/360) // 角度转弧度的转换率
#define RAD_TO_DEG 57.295779513082320876798154814105   // 弧度转角度的转换率
#define DEG_TO_RAD 0.01745329251994329576923690768489 // 角度转弧度的转换率
     

// TODO: Make calibration routine
void pose_update_entry(void* param);

void pose_init(void) 
{
	
	kalman_init(&kalman_x);
	kalman_init(&kalman_y);
	
	mpu6050 = mpu6xxx_init( "i2c1", RT_NULL);
	
	mpu6xxx_get_accel(mpu6050, &accel);
	mpu6xxx_get_gyro(mpu6050, &gyro);
	mpu6xxx_get_temp(mpu6050, &temp);
	
	acc_x = (int16_t)accel.x;
	acc_y = (int16_t)accel.y;
	acc_z = (int16_t)accel.z;

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
	double pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	kalman_set_angle(&kalman_x , roll);
	kalman_set_angle(&kalman_y , pitch);

	gyro_x_angle = roll;
	gyro_y_angle = pitch;
	comp_angle_x = roll;
	comp_angle_y = pitch;
	
	pose_thread = rt_thread_create("pose_cal",pose_update_entry,RT_NULL , 1024 , 4 , 10);
	
	if(pose_thread)
	{
		rt_thread_startup(pose_thread);
	}
	
	timer = rt_tick_get();
}
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(pose_init,  pose_init);
#endif


void pose_update_entry(void* param)
{
	while(1)
	{
		/* Update all the values */
		mpu6xxx_get_accel(mpu6050, &accel);
		mpu6xxx_get_gyro(mpu6050, &gyro);
		mpu6xxx_get_temp(mpu6050, &temp);
	
		acc_x = (int16_t)accel.x;
		acc_y = (int16_t)accel.y;
		acc_z = (int16_t)accel.z;
 
		temp_raw = (int16_t)temp;
		
		gyro_x = (int16_t)gyro.x;
		gyro_y = (int16_t)gyro.y;
		gyro_z = (int16_t)gyro.z;

		double dt = (double)(rt_tick_get() - timer) / 1000; // Calculate delta time
		timer = rt_tick_get();

		// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
		// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
		// It is then converted from radians to degrees
		#ifdef RESTRICT_PITCH // Eq. 25 and 26
		double roll  = atan2(acc_y, acc_z) * RAD_TO_DEG;
		double pitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;
		#else // Eq. 28 and 29
		double roll  = atan(acc_y / sqrt(acc_x * acc_X + acc_z * acc_z)) * RAD_TO_DEG;
		double pitch = atan2(-acc_x, acc_z) * RAD_TO_DEG;
		#endif

		double gyro_x_rate = gyro_x / 131.0; // Convert to deg/s
		double gyro_y_rate = gyro_y / 131.0; // Convert to deg/s

		#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kal_angle_x > 90) || (roll > 90 && kal_angle_x < -90)) 
		{
			kalman_set_angle(&kalman_x , roll);
			comp_angle_x = roll;
			kal_angle_x = roll;
			gyro_x_angle = roll;
		} 
		else
		{
			// Calculate the angle using a Kalman filter
			kal_angle_x = kalman_get_angle(&kalman_x , roll , gyro_x_rate , dt);
		}
		if (fabs(kal_angle_x) > 90)
		{
			gyro_y_rate = -gyro_y_rate; // Invert rate, so it fits the restriced accelerometer reading
		}
		
		kal_angle_y = kalman_get_angle(&kalman_y , pitch , gyro_y_rate, dt);
		 
		#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if((pitch < -90 && kal_angle_y > 90) || (pitch > 90 && kal_angle_y < -90)) 
		{
			kalman_set_angle(&kalman_y , pitch);
			comp_angle_y = pitch;
			kal_angle_y = pitch;
			gyro_y_angle = pitch;
		} 
		else
		{
			// Calculate the angle using a Kalman filter
			kal_angle_y = kalman_get_angle(&kalman_y , pitch ,gyro_y_rate , dt);
		
		}
		if(abs(kal_angle_y) > 90)
		{
			gyro_x_rate = -gyro_x_rate; // Invert rate, so it fits the restriced accelerometer reading
		}
		// Calculate the angle using a Kalman filter
		kal_angle_x = kalman_get_angle(&kalman_x , roll ,gyro_x_rate , dt);
		#endif

		gyro_x_angle += gyro_x_rate * dt; // Calculate gyro angle without any filter
		gyro_y_angle += gyro_y_rate * dt;
		//gyro_x_angle += kalman_get_rate(&kalman_x) * dt; // Calculate gyro angle using the unbiased rate
		//gyro_y_angle += kalman_get_rate(&kalman_y) * dt;

		comp_angle_x = 0.93 * (comp_angle_x + gyro_x_rate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		comp_angle_y = 0.93 * (comp_angle_y + gyro_y_rate * dt) + 0.07 * pitch;

		// Reset the gyro angle when it has drifted too much
		if (gyro_x_angle < -180 || gyro_x_angle > 180)
		{
			gyro_x_angle = kal_angle_x;
		}
		if (gyro_y_angle < -180 || gyro_y_angle > 180)
		{
			gyro_y_angle = kal_angle_y;
		}
 
		LOG_D("X  (%f\t%f\t%f\t%f)", roll ,gyro_x_angle , comp_angle_x , kal_angle_x);  
		LOG_D("Y  (%f\t%f\t%f\t%f)\r\n", pitch ,gyro_y_angle,comp_angle_y , kal_angle_y);  
 		
		rt_thread_mdelay(10);

	}
}





