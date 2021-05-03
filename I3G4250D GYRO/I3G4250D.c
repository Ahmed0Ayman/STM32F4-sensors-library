/*
 * GYRO.c
 *
 *  Created on: Apr 1, 2021
 *      Author: ahmed ayman
 */

#include "MEMS.h"

////////////// the following private variable used by this library to pass data between functions //////////////
static GYRO_DATA_t GYROReadedDataSTATIc;
static GYRO_DATA_t GYROReadedDataPrev;
static _Bool GYRO_IS_CALLIBRATED ;
static int32_t GYROREADING_OFFCSET[3];

////// defined as 1 in case of debugging mode to see these value in stm debug monitor //////////////////
#define __DEBUG__  1




#if(__DEBUG__)


// readed value from the gyro in 2's complement 16 bit
static int16_t MEMAS_ROW_DATA_X = 0 ;
static int16_t MEMAS_ROW_DATA_Y = 0 ;
static int16_t MEMAS_ROW_DATA_Z = 0 ;

// calculated angles of measured value
static float MEMAS_ANGLE_X;
static float MEMAS_ANGLE_Y;
static float MEMAS_ANGLE_Z;

// calculation of Euler's angle
static float GYRO_PITCH ;
static float GYRO_ROLL ;
static float GYRO_YAW ;

#endif

/*****************************************************************************************
 *  private function return ALL Sensor values(x,y,z.temp)
 *  parameter : Config_Struct  UPData values(x,y,z.temp) in GYROreadedata if it is successfully
 *  return GYRO_OK if this function successfully set registers
 ****************************************************************************************/

static void GYRO_Read_ALL_REG(void);

/*****************************************************
 * brief  @ this function Set_Sngle_Reg used to Read the data from specific register in the gyro
 * param. @ RegAddr : register address
 * param. @ Data    : data that you want to assign to that address
 * return void
 */

void Set_Sngle_Reg(uint32_t RegAddr, uint8_t *Data) {
	uint8_t First_Add;
	First_Add = GYRO_Write_Single_ADD(RegAddr);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);                   // CS
	HAL_SPI_Transmit(&hspi5, &First_Add, 1, 1);
	HAL_SPI_Transmit(&hspi5, (uint8_t*) Data, 1, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

}

/*****************************************************
 * brief  @ this function Read_Sngle_Reg used to set  data byte from specific register in the gyro
*  param. @  RegAddr : register address
 * param. @  Data    : data that you want to hold the data that returned from that address
 * return void
 */

void Read_Sngle_Reg(uint32_t RegAddr, uint8_t *Data) {
	uint8_t First_Add;
	First_Add = GYRO_Read_Single_ADD(RegAddr);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);                   // CS
	HAL_SPI_Transmit(&hspi5, &First_Add, 1, 1);
	HAL_SPI_Receive(&hspi5, (uint8_t*) Data, 1, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

}

/*****************************************************
 *brief  @ this function Set_Multi_Reg used to Read the data from specific register in the gyro
 * param. @  RegAddr : register address
 * param. @  Data    : data that you want to assign to that address
 * param. @  NumOfReg: number of contiguous register you want to write data
 * return void
 */

void Set_Multi_Reg(uint32_t RegAddr, uint8_t *Data, uint32_t NumOfReg) {
	uint8_t First_Add;
	First_Add = GYRO_Write_MULT_ADD(RegAddr);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);                   // CS
	HAL_SPI_Transmit(&hspi5, &First_Add, 1, 1);
	HAL_SPI_Transmit(&hspi5, (uint8_t*) Data, NumOfReg, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

}

/*****************************************************
 * brief  @ this  function Read_Multi_Reg used to set  NUM. of bytes start from RegAddr register in the gyro
 * param. @  RegAddr  : register address
 * param. @  Data     :var add that you want to hold the data that returned from that address
 * param. @  NumOfReg :number of contiguous register you want to collect data
 * return void
 */

void Read_Multi_Reg(uint32_t RegAddr, uint8_t *Data, uint8_t NumOfReg) {
	uint8_t First_Add;
	First_Add = GYRO_Read_MULT_ADD(RegAddr);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);// CS
	HAL_SPI_Transmit(&hspi5,&First_Add,1,1);
	HAL_SPI_TransmitReceive(&hspi5, (uint8_t*) Data, (uint8_t*) Data, NumOfReg,
			1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);

}


/*****************************************************
 * brief  @ this  function GYRO_X_GYRO_Read used to Read X_data register form gyro
 * param. @ X_Axis_Var : pointer to the var that will hold the readed value
 * return void
 */


void GYRO_X_GYRO_Read(int16_t *X_Axis_Var) {

	uint8_t status =0;
	if (GYRO_IS_CALLIBRATED == false)
		GYRO_Callibration_Fun();

	while (1) {
		Read_Sngle_Reg(GYRO_STATUS_REG_ADR, &status);
		if ((status & GYRO_STATUS_REG_XDA)) {
			break;
		}
	}

	Read_Multi_Reg(GYRO_OUT_X_LOW_ADR,
			(uint8_t*)X_Axis_Var, 2);


}

/*****************************************************
 * brief  @ this  function GYRO_Y_GYRO_Read used to Read Y_data register form gyro
 * param. @ Y_Axis_Var : pointer to the var that will hold the readed value
 * return void
 */

void GYRO_Y_GYRO_Read(int16_t *Y_Axis_Var) {

	uint8_t status =0;
	if (GYRO_IS_CALLIBRATED == false)
		GYRO_Callibration_Fun();

	while (1) {
		Read_Sngle_Reg(GYRO_STATUS_REG_ADR, &status);
		if ((status & GYRO_STATUS_REG_YDA)) {
			break;
		}
	}

	Read_Multi_Reg(GYRO_OUT_Y_LOW_ADR,
			(uint8_t*)Y_Axis_Var, 2);
}


/*****************************************************
 * brief  @ this  function GYRO_Z_GYRO_Read used to Read Z_data register form gyro
 * param. @ Z_Axis_Var : pointer to the var that will hold the readed value
 * return void
 */

void GYRO_Z_GYRO_Read(int16_t *Z_Axis_Var) {
	uint8_t status =0;
	if (GYRO_IS_CALLIBRATED == false)
		GYRO_Callibration_Fun();

	while (1) {
		Read_Sngle_Reg(GYRO_STATUS_REG_ADR, &status);
		if ((status & GYRO_STATUS_REG_ZDA)) {
			break;
		}
	}

	Read_Multi_Reg(GYRO_OUT_Z_LOW_ADR,
			(uint8_t*)Z_Axis_Var, 2);
}

/*****************************************************
 * brief  @ this  function GYRO_Temp_Read used to Read Temp_register form gyro
 * param. @ Temp_Var : pointer to the var that will hold the readed value
 * return void
 */

void GYRO_Temp_Read(int16_t *Tmep_Var) {

	*Tmep_Var = (int32_t) GYROReadedDataSTATIc.TEMP;
}



///// this function used to calculate Euler's angle (pitch , row , yaw )
void GYRO_ALL_Angles_Read(GYRO_ANGLES_t * Struct_ALL_ANGELS) {
	 static uint32_t Tick = 0;
	 float DT =0;      // these calculation is depend on accumulation so this variable used to hold differential in time

	if (GYRO_IS_CALLIBRATED == false)
		GYRO_Callibration_Fun(); // check if gyro is calibrated


	GYRO_Read_ALL_REG();   // read all gyro data
	DT = abs(HAL_GetTick() - Tick)/1000.0 ;   // calculate delta T

	// now we are read to calculate angles
	Struct_ALL_ANGELS->GYRO_ANGLE_PITCH +=((int)((((GYROReadedDataSTATIc.X_Value-GYROREADING_OFFCSET[0] )*(GYRO_SEN_2000_dps) + GYROReadedDataPrev.X_Value)*(DT))*100))/100.0;
	Struct_ALL_ANGELS->GYRO_ANGLE_ROLL +=((int)((((GYROReadedDataSTATIc.Y_Value-GYROREADING_OFFCSET[1] ) *(GYRO_SEN_2000_dps) +GYROReadedDataPrev.Y_Value)*(DT))*100))/100.0;
	Struct_ALL_ANGELS->GYRO_ANGLE_YAW +=((int)((((GYROReadedDataSTATIc.Z_Value-GYROREADING_OFFCSET[2] ) *(GYRO_SEN_2000_dps) + GYROReadedDataPrev.Z_Value) *(DT))*100))/100.0;

	Tick = HAL_GetTick();
	GYROReadedDataPrev.X_Value =(GYROReadedDataSTATIc.X_Value-GYROREADING_OFFCSET[0] ) *(GYRO_SEN_2000_dps) ;
	GYROReadedDataPrev.Y_Value =(GYROReadedDataSTATIc.Y_Value-GYROREADING_OFFCSET[1] ) *(GYRO_SEN_2000_dps)  ;
	GYROReadedDataPrev.Z_Value = (GYROReadedDataSTATIc.Z_Value-GYROREADING_OFFCSET[2] ) *(GYRO_SEN_2000_dps) ;

#if(__DEBUG__) // used in debug mode only otherwise must be removed by preprocessor step
	MEMAS_ANGLE_X =  GYROReadedDataPrev.X_Value ;
	MEMAS_ANGLE_Y =  GYROReadedDataPrev.Y_Value ;
	MEMAS_ANGLE_Z =  GYROReadedDataPrev.Z_Value ;

	GYRO_PITCH = Struct_ALL_ANGELS->GYRO_ANGLE_PITCH;
	GYRO_ROLL  = Struct_ALL_ANGELS->GYRO_ANGLE_ROLL;
	GYRO_YAW   = Struct_ALL_ANGELS->GYRO_ANGLE_YAW;
#endif
}


// this function is called first time only to calculate the mean offset
void GYRO_Callibration_Fun(void) {
	GYRO_IS_CALLIBRATED = false;
	for (uint32_t i = 0; i < 2000; i++) {
		GYRO_Read_ALL_REG();
		GYROREADING_OFFCSET[0] += ((GYROReadedDataSTATIc.X_Value) );

		GYROREADING_OFFCSET[1] += (((GYROReadedDataSTATIc.Y_Value)) );

		GYROREADING_OFFCSET[2] += (((GYROReadedDataSTATIc.Z_Value)));
	}
	////////////////// calculate the mean offset /////////////////////
	GYROREADING_OFFCSET[0] = GYROREADING_OFFCSET[0] / 2000;
	GYROREADING_OFFCSET[1] = GYROREADING_OFFCSET[1] / 2000;
	GYROREADING_OFFCSET[2] = GYROREADING_OFFCSET[2] / 2000;

	GYRO_IS_CALLIBRATED = true;  // set to indicate  the gyro is passed initialization state end now ready to read

}
/****************************************************************************************
 * set all configurations
 *  parameter : Config_Struct  UPData configuration registers value
 *  return GYRO_OK if this function successfully set registers
 *  you can add and remove any registers that you need to config
 ****************************************************************************************/
GYRO_status_t GYRO_Config(GYROConfig_t *Config_Struct) {

	GYROConfig_t RECV_CONFIG = { 0 };  // used only for check successfully configuration
	Set_Multi_Reg(GYRO_CTRL_REG1_ADR, (uint8_t*) Config_Struct, 5);
	Set_Sngle_Reg(GYRO_INT1_CFG_ADR,
			(uint8_t*) &(Config_Struct->GYRO_INT1_CFG));

	/* check if  module start to collect data successfully we can check that trough status register */

	Read_Multi_Reg(GYRO_CTRL_REG1_ADR, (uint8_t*) Config_Struct, 5);

	if ((RECV_CONFIG.GYRO_CTRL_REG1) != Config_Struct->GYRO_CTRL_REG1) // compare  check for first byte contain enable bit
		return GYRO_ERROR;
	return GYRO_OK;

}

/*****************************************************************************************
 *  return ALL Sensor data register value values(x,y,z.temp)
 *  parameter : Config_Struct  UPData values(x,y,z.temp) in GYROreadedata if it is successfully
 *  return GYRO_OK if this function successfully set registers
 ****************************************************************************************/

static void GYRO_Read_ALL_REG(void) {

	uint8_t status =0;
	uint8_t data[6] ={0};

	while (1) {        // wait until data become ready
		Read_Sngle_Reg(GYRO_STATUS_REG_ADR, &status);
		if ((status & GYRO_STATUS_REG_ZYXDA)) {
			break;
		}
	}

	Read_Multi_Reg(GYRO_OUT_X_LOW_ADR, (uint8_t*)data, 8);  // read 6 byte data from gyro
	Read_Sngle_Reg(GYRO_OUT_TEMP_ADR, (uint8_t*)&(GYROReadedDataSTATIc.TEMP));
#if(__DEBUG__)
	MEMAS_ROW_DATA_X = data[1]<<8 | data[0] ;
	MEMAS_ROW_DATA_Y = data[3]<<8 | data[2] ;
	MEMAS_ROW_DATA_Z = data[5]<<8 | data[4] ;
#endif
	GYROReadedDataSTATIc.X_Value  = data[1]<<8 | data[0] ;
	GYROReadedDataSTATIc.Y_Value  = data[3]<<8 | data[2] ;
	GYROReadedDataSTATIc.Z_Value  = data[5]<<8 | data[4] ;

}

