/*
 * GYRO.h
 *
 *  Created on: Apr 1, 2021
 *      Author: ahmed ayman
 */

/*************************************
 * recap on this module
 * this module connected on discovery on SPI5
 * GYRO_CLK  ->> PF7
 * GYRO_MISO ->> PF8
 * GYRO_MOSI ->> PF9
 * GYRO_CS   ->> PC1
 *
 * THIS MODULE SUPPORT TWO COMMUNICATION PROTOCOLS SPI& I2C
 *
 * Register width 8 bits (1 bye)
 *
 * maximum SPI 10MHZ
 * maximum I2C 100,400 khz
 *
 * contain tamp. sensor (-40c - 80 c)
 *
 * stm32f4 contain 2 dma modules for best performance and to free the processor form
 * stm32f4 interrupt overhead
 *
 * before you need to make gyro in stable table to calculate better offset
 * how to use this library
 * first configure spi peripheral  then call GYRO_Config function with
 * parameter of type GYROConfig_t this hold the configuration
 * then call GYRO_ALL_Angles_Read so you will get the angles and can project then in a debugger
 * or any other useful usage
 *
 *
 ***************************************/



#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include <stdint.h>
#include <stm32f429xx.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include <math.h>




#define GYRO_SEN_2000_dps      (0.07f)
#define GYRO_SEN_500_dps       (0.0175f)
#define GYRO_SEN_250_dps       (0.00875f)



extern SPI_HandleTypeDef hspi5;



/*******************
 * you must use the following struct to initialize the GYRO module
 *
 */


/* there is only five contiguous configuration register that need to set by user
before start access the gyro data register so we put all these registers in one struct
*/
typedef struct {
	uint8_t GYRO_CTRL_REG1 ;
	uint8_t GYRO_CTRL_REG2 ;
	uint8_t GYRO_CTRL_REG3 ;
	uint8_t GYRO_CTRL_REG4 ;
	uint8_t GYRO_CTRL_REG5 ;
	uint8_t GYRO_INT1_CFG  ;

}GYROConfig_t;



// this struct hold all gyro angles
typedef struct {

	float  GYRO_ANGLE_PITCH ;
	float  GYRO_ANGLE_ROLL ;
	float  GYRO_ANGLE_YAW ;
	float  TEMP_IN_CEL ;
}GYRO_ANGLES_t;


// enum used to indicate the stauts of the operation
typedef enum {
	GYRO_OK,
	GYRO_ERROR
}GYRO_status_t;

/*
 * this struct used to hold all 16 bit 2's complement data readed from gyro
 */
typedef struct {

	int16_t  X_Value ;
	int16_t  Y_Value ;
	int16_t  Z_Value ;
	int8_t  TEMP ;
}GYRO_DATA_t;


// each angle specified as a bit so we will check he availabilty by anding with position
typedef enum
{
	GYRO_Not_READY = 0,           // the anding with this flag retun zero means their is data read yet
	GYRO_X_Is_READY = 1,          // set first bit as a x_axis is ready for read
	GYRO_Y_Is_READY = 2,          //  set second bit as a Y_axis is ready for read
	GYRO_Z_Is_READY = 4,          // set third bit as a Z_axis is ready for read
	GYRO_XYZ_Is_READY = 7,        // set first three bits as a XYZ_axis is ready for read
}GYRO_READ_STATUS;



/****************************************************************
 * chip control macros used to control the direction and the size of data
 * transfered by SPI
 ***************************************************************/
#define READ_BIT_CONTROL                    (0x80)
#define WRITE_BIT_CONTROL                   (0x00)
#define MULT_ADD_BIT_CONTROL                (0x40)
#define SINGLE_ADD_BIT_CONTROL              (0x00)


/////////////////// four combinations of possibilities///////////////////////////////////////////
#define GYRO_Read_MULT_ADD(ADDR)         ((ADDR | READ_BIT_CONTROL | MULT_ADD_BIT_CONTROL ))
#define GYRO_Read_Single_ADD(ADDR)       ((ADDR | READ_BIT_CONTROL | SINGLE_ADD_BIT_CONTROL))

#define GYRO_Write_MULT_ADD(ADDR)        ((ADDR | WRITE_BIT_CONTROL | MULT_ADD_BIT_CONTROL ))
#define GYRO_Write_Single_ADD(ADDR)      ((ADDR | WRITE_BIT_CONTROL | SINGLE_ADD_BIT_CONTROL ))



/*****************************************************************
 * different possibilities for GYRO_CTRL_REG1  @ 0x20
 *****************************************************************/

#define GYRO_CTRL_REG1_ODR_HZ_200            0x40
#define GYRO_CTRL_REG1_ODR_HZ_400            0x80
#define GYRO_CTRL_REG1_ODR_HZ_800            0xc0

#define GYRO_CTRL_REG1_POWER_EN              0x08    // power enable
#define GYRO_CTRL_REG1_ZEN                   0x04    // Z axes enable
#define GYRO_CTRL_REG1_YEN                   0x02    // Y axes enable
#define GYRO_CTRL_REG1_XEN                   0x01    // x axes enable
/****************************************************************/


/*****************************************************************
 * different possibilities for GYRO_CTRL_REG2  @ 0x21
******************************************************************/
#define GYRO_CTRL_REG2_NORMAL_MODE                    0x20
#define GYRO_CTRL_REG2_HPCF_AT_ODR_200       0x00
#define GYRO_CTRL_REG2_HPCF_AT_ODR_400       0x00
#define GYRO_CTRL_REG2_HPCF_AT_ODR_800       0x00
/****************************************************************/


/*****************************************************************
 * different possibilities for GYRO_CTRL_REG3  @ 0x22
******************************************************************/
#define GYRO_CTRL_REG3_INT1_EN               0x80     // ENABLE INT ON INT1
#define GYRO_CTRL_REG3_INT1_BOOT_MODe        0x40      // BOOT STATE AVAIL. ON INT1
#define GYRO_CTRL_REG3_INT1_ACT_STATE        0x20     // SET ACTIVE STAE AT LOW
#define GYRO_CTRL_REG3_INT2_EN               0x08     // INT2 INTERRUPT ON DATA READY
#define GYRO_CTRL_REG3_FIFO_WTM              0X04    // INTERRUPT ON INT2 WHEN FIFO WATEMWRK
#define GYRO_CTRL_REG3_FIFO_ORUN             0x02   // INTRRUPT ON INT2 WHEN FIFO OVERRUN
#define GYRO_CTRL_REG3_FIFO_EMPT             0x01   // INTERRUPT ONINT2 WHEN FIFO EMPTY
/****************************************************************/


/*****************************************************************
 * different possibilities for GYRO_CTRL_REG4  @ 0x23
******************************************************************/

#define GYRO_CTRL_REG4_BLE                    0X40
#define GYRO_CTRL_REG4_FS1_2000               0X30
#define GYRO_CTRL_REG4_FS2_2000               0X20
#define GYRO_CTRL_REG4_FS_500                 0X10
#define GYRO_CTRL_REG4_FS_250                 0X00         // DEFAULT VALUE
#define GYRO_CTRL_REG4_3WIRE_MODE             0X01         // USE SPI 3 WIRE MODE
/*******************************************************************/


/*****************************************************************
 * different possibilities for GYRO_CTRL_REG5  @ 0x24
************************************************************************/

#define GYRO_CTRL_REG5_BOOT                   0X80         //REBOOT MOMORY
#define GYRO_CTRL_REG5_FIFO_EN                0X40         //FIFO ENABLE
#define GYRO_CTRL_REG5_HPFILTER_EN            0X11         //HIFG PAS SFILER EN
#define GYRO_CTRL_REG5_LPF_EN                 0x02
/**********************************************************************/

/**********************************************************************
 * 	status register @ 0x27
 ********************************************************************/

#define GYRO_STATUS_REG_ZXYOR                 0X80        // X,Y,Z OVERRUN
#define GYRO_STATUS_REG_ZOR                   0X40
#define GYRO_STATUS_REG_YOR                   0X20
#define GYRO_STATUS_REG_XOR                   0X10
#define GYRO_STATUS_REG_ZYXDA                 0X08        // X,Y,Z DATA AVAILABLE
#define GYRO_STATUS_REG_ZDA                   0X04
#define GYRO_STATUS_REG_YDA                   0X02
#define GYRO_STATUS_REG_XDA                   0X01
/*************************************************************************/


/**********************************************************************
 *  FIFO CONTROL REGISTER @ 0x2E
 ********************************************************************/
#define GYRO_FIFO_CTRL_REG_FM_BYPASS          0X00
#define GYRO_FIFO_CTRL_REG_FM_FIFOMD          0X20
#define GYRO_FIFO_CTRL_REG_FM_STREAM          0X40
#define GYRO_WATER_MARK_THREADSHOLD_POS       0X00    // USE AS SHIFT VALUE FOR SETTING WATERMAR THREADSHOLD
/********************************************************************/


/**********************************************************************
 *  FIFO STATUS  REGISTER @ 0x2F
 ********************************************************************/
#define GYRO_FIFO_SR_WTM                      0X80
#define GYRO_FIFO_SR_OVRN                     0X40
#define GYRO_FIFO_SR_EMPTY                    0X20
#define GYRO_FIFO_SR_FSS_MASK                 0X1F   // USE AS A MASK
/**********************************************************************/


/**********************************************************************
 *  INTERRUPT STATUS  REGISTER @ 0x2F
 ********************************************************************/
#define GYRO_INT1_SR_INTACTIVE              0X40
#define GYRO_INT1_SR_ZH_EVENT               0X20
#define GYRO_INT1_SR_ZL_EVENT               0X10
#define GYRO_INT1_SR_YH_EVENT               0X08
#define GYRO_INT1_SR_YL_EVENT               0X04
#define GYRO_INT1_SR_XH_EVENT               0X02
#define GYRO_INT1_SR_XL_EVENT               0X01
/********************************************************************/









/**************************************************************************
 * REFERANCE VALUE FOR DATA GENERATED   @  0X25
 * OUT TEMP.                            @  0X26
 * OUT X_LOW                            @  0X28
 * OUT X_HIGH                           @  0X29
 * OUT Y_LOW                            @  0X2A
 * OUT Y_HIGH                           @  0X2B
 * OUT Z_LOW                            @  0X2C
 * OUT Z_HIGH                           @  0X2D
 **************************************************************************/
#define    GYRO_OUT_X_LOW_ADR                0X28
#define    GYRO_OUT_X_HIGH_ADR               0X29
#define    GYRO_OUT_Y_LOW_ADR                0X2A
#define    GYRO_OUT_Y_HIGH_ADR               0X2B
#define    GYRO_OUT_Z_LOW_ADR                0X2C
#define    GYRO_OUT_Z_HIGH_ADR               0X2D
#define    GYRO_OUT_TEMP_ADR                 0x26
/***************************************************************************/

///////////////////// 5 configuration registers /////////////////////////////////////////
#define GYRO_CTRL_REG1_ADR                   0x20
#define GYRO_CTRL_REG2_ADR                   0x21
#define GYRO_CTRL_REG3_ADR                   0x22
#define GYRO_CTRL_REG4_ADR                   0x23
#define GYRO_CTRL_REG5_ADR                   0x24
////////////////////////////////////////////////////////////////////////////////////////////
#define GYRO_STATUS_REG_ADR                  0x27    // status register
#define GYRO_FIFO_SR_ADR                     0x2F
#define GYRO_INT1_CFG_ADR                    0x30
#define GYRO_INT1_SR_ADR                     0x31

/*****************************************************************************/


/******************************************************************************
 * the follow APIs used to communicate with he module
 * we transmit the data through Interrupt APIs
 *
 ******************************************************************************/


/****************************************************************************************
 * set all configurations
 *  parameter : Config_Struct  UPData configuration registers value
 *  return GYRO_OK if this function successfully set registers
 ****************************************************************************************/
 GYRO_status_t GYRO_Config(GYROConfig_t * Config_Struct );

void Set_Sngle_Reg(uint32_t RegAddr,uint8_t * Data);

void Read_Sngle_Reg(uint32_t RegAddr,uint8_t * Data);

void Set_Multi_Reg(uint32_t RegAddr,uint8_t * Data,uint32_t NumOfReg);

void Read_Multi_Reg(uint32_t RegAddr,uint8_t * Data,uint8_t NumOfReg);

void GYRO_X_GYRO_Read(int16_t * X_Axis_Var);

void GYRO_Y_GYRO_Read(int16_t * X_Axis_Var);

void GYRO_Z_GYRO_Read(int16_t * X_Axis_Var);

void GYRO_Temp_Read(int16_t * Tmep_Var);

void GYRO_ALL_Angles_Read(GYRO_ANGLES_t * Struct_ALL_ANGELS);

GYRO_READ_STATUS MEME_IsDataReady(void);

void GYRO_Callibration_Fun(void);


#endif /* INC_GYRO_H_ */
