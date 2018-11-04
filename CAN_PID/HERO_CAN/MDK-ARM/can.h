/*
Please refer to can.c for documentation.
*/

#ifndef __CAN_H
#define __CAN_H

/* ========== includes ========== */
#include "main.h"
#include "stm32f4xx_hal.h"
/* ===========Defination=========== */
#define SPEED_OUTPUT_ATTENUATION (1.0f)
#define RATE_BUF_SIZE 6
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	80.0f,\
	3.0f,\
	5.0f,\
	0,\
	0,\
	0,\
	5000,\
	0,\
	800,\
	&PID_Calc,\
	&PID_Reset,\
}\
/* ========== headers ========== */
extern CAN_HandleTypeDef hcan1;
/* ============================= */
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	float rate_buf[RATE_BUF_SIZE];	//buf，for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;				//角度
	float last_ecd_angle;
	float ecd_speed;
}Encoder;       

typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float output;
	float last_output;
	float kp;
	float ki;
	float kd;
	float KpComponent;
	float KiComponent;
	float KdComponent;
	float output_limit;
	float windup_limit;
	float max_step;
	void (*Calc)(struct PID_Regulator_t*);
	void (*Reset)(struct PID_Regulator_t*);
	float Ki_Limit;
}PID_Regulator_t;
/* ========== ids ========== */
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID 					0x207
#define CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID 					0x208
/* ========================= */



/* ========== error handling ========== */
extern void _Error_Handler(char *, int);
/* ==================================== */

/* ========== CAN initialize functions ========== */
void CAN_Initialize(void);
/* ============================================== */

/* ========== CAN user/transmit functions ========== */
void CAN_SendMsg(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *canTxHeader,uint8_t* canMsg);
void set_CM_speed(int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq);
/* ===========Speed calculation=========== */
void PID_Calc(PID_Regulator_t *pid); 
void EncoderProcess(volatile Encoder *v, uint8_t* msg);
void GetEncoderBias(volatile Encoder *v,CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg);
void PID_Reset(PID_Regulator_t *pid);
void Motor_Task(void);
/* ========== CAN receive functions ========== */
void CanReceiveMsgProcess(CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg);
/* ===========Speed calculation=========== */

#endif
