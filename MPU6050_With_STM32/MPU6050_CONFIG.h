/*
 * MPU6050_CONFIG.h
 *
 *  Created on: Apr 8, 2021
 *      Author: Ahmed_Ayman
 */
/* you need to pay some attention when you made changes in this file
 * this file hold all the required configurations that you need to
 * config your MPU before use but here is the default configuration and I made this library with heavy comments to
 * to make its easy for you to know what is evey line in the code is made to so
 * so its your call to use this default or change it's depending on your application
 *
 *
 *
 *  */
#ifndef INC_MPU6050_CONFIG_H_
#define INC_MPU6050_CONFIG_H_

#define  MPU6050_GYRO_SCALL    500u        // in degree/sec.
#define  MPU6050_ACCEL          2u         // in g
#define  MPU_REFRUSH_READING_RATE  5      // in m sec   we need accurate time interval must be measured through oscilloscope



#define RadianToDegree   57.295f
#define DegreeToRadian   0.0174f

/* here if you need to change thew gain of the complemantary filter you can do this here */
#define ComplemantaryFilterGain1  0.96f
#define ComplemantaryFilterGain2  0.04f



/* now this #defines will include the proper configuration depend on your choose of configurations */
#if(MPU6050_GYRO_SCALL == 250)
#define MPU_SCALL 131.0f
#elif(MPU6050_GYRO_SCALL == 500)
#define MPU_SCALL 65.5f
#elif(MPU6050_GYRO_SCALL == 1000)
#define MPU_SCALL 32.8f
#elif(MPU6050_GYRO_SCALL == 2000)
#define MPU_SCALL 16.4f

#endif


#if(MPU6050_ACCEL == 2)
#define MPU6050_ACCEL_SCALL 16385.0f

#elif(MPU6050_ACCEL == 4)
#define MPU6050_ACCEL_SCALL  8192.0f

#elif(MPU6050_ACCEL == 8)
#define MPU6050_ACCEL_SCALL 4096.0f

#elif(MPU6050_ACCEL == 16)
#define MPU6050_ACCEL_SCALL 2048.0f

#endif


#endif /* INC_Mne PU6050_CONFIG_H_ */
