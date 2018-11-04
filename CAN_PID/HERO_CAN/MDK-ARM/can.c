/*
IMPLEMENTATION:
Four files are needed to implement CAN
1. can.c
2. can.h
File 1 and 2 contain HAL CAN drivers that is placed into two files so that all
the functions are same when used on different systems. Moreover, file 1 and 2 also
contain user functions etc.
*/

/* ========== includes ========== */
#include "can.h"
/* ============================== */

/* ========== headers ========== */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef can1TxHeader0;
CAN_TxHeaderTypeDef can1TxHeader1;
CAN_RxHeaderTypeDef can1RxHeader;
/* ============================= */

/* ========== filters ========== */
CAN_FilterTypeDef can1Filter;
/* ============================= */

/* ========== buffers ========== */
uint8_t canTxMsg0[8] = {0};
uint8_t canTxMsg1[8] = {0};
uint32_t can_count=0;
/* ============================= */
volatile Encoder CM1Encoder;
volatile Encoder CM2Encoder;
volatile Encoder CM3Encoder;
volatile Encoder CM4Encoder;  
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
/* ========== CAN initialize functions ========== */
void CAN_Initialize(void)
{
	hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	can1TxHeader0.IDE = CAN_ID_STD;
	can1TxHeader0.StdId = 0x200;
	can1TxHeader0.DLC = 8;
	
	can1TxHeader1.IDE = CAN_ID_STD;
	can1TxHeader1.StdId = 0x1FF;
	can1TxHeader1.RTR = CAN_RTR_DATA;
	can1TxHeader1.DLC = 8;
	
	
	can1Filter.FilterActivation = ENABLE;
	can1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can1Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can1Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can1Filter.FilterIdHigh = 0x0000;
	can1Filter.FilterIdLow = 0x0000;
	can1Filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1,&can1Filter);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
	HAL_CAN_Start(&hcan1);
}
/* ============================================== */

/* ========== CAN user/transmit functions ========== */
void CAN_SendMsg(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *canTxHeader,uint8_t* canMsg)
{
	HAL_CAN_AddTxMessage(hcan,canTxHeader,canMsg,(void*)CAN_TX_MAILBOX0);
}
// use set_CM_speed for chassis test. However, this function will soon be degraded >:)
void set_CM_speed(int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq)
{
    canTxMsg0[0] = (uint8_t)(cm1_iq >> 8);
    canTxMsg0[1] = (uint8_t)cm1_iq;
    canTxMsg0[2] = (uint8_t)(cm2_iq >> 8);
    canTxMsg0[3] = (uint8_t)cm2_iq;
    canTxMsg0[4] = (uint8_t)(cm3_iq >> 8);
    canTxMsg0[5] = (uint8_t)cm3_iq;
    canTxMsg0[6] = (uint8_t)(cm4_iq >> 8);
    canTxMsg0[7] = (uint8_t)cm4_iq;
    CAN_SendMsg(&hcan1,&can1TxHeader0,canTxMsg0);
}
/* ================================================= */

/* ========== CAN receive functions ========== */
void CanReceiveMsgProcess(CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{      

	can_count++;
		switch(rxHeader->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{       
          (can_count<=50) ? GetEncoderBias(&CM1Encoder ,rxHeader,msg):EncoderProcess(&CM1Encoder ,msg);					
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM2Encoder ,rxHeader,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM3Encoder ,rxHeader,msg):EncoderProcess(&CM3Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM4Encoder ,rxHeader,msg):EncoderProcess(&CM4Encoder ,msg);
				}
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
				}break;
				
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{	
				}break;		
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
				}break;
				case CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID:
				{
				}break;
				
		

}
}
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

	//calculate PID
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KiComponent += pid->ki * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	
	//output value limit
	if((pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
}
void EncoderProcess(volatile Encoder *v, uint8_t* msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg[0]<<8)|msg[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(can_count < 50)
	{
		v->ecd_raw_rate = 0;
	}
	else
	{
		if(v->diff < -7000)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff>7000)
		{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff- 8192;
		}		
		else
		{
			v->ecd_raw_rate = v->diff;
		}
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	//v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192.0f;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		

}
void GetEncoderBias(volatile Encoder *v,CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{

            v->ecd_bias = (msg[0]<<8)|msg[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}
void PID_Reset(PID_Regulator_t *pid)
{
	pid->ref = 0;
	pid->fdb = 0;
	pid->output = 0;
}

void Motor_Task(void)
{
	CM1SpeedPID.ref = 600;
	CM2SpeedPID.ref = 600;
	CM3SpeedPID.ref = 600;
	CM4SpeedPID.ref = 600;
	int counter=0;
	while (1)
  {

  CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
  PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
		/* USER CODE BEGIN 3 */
		HAL_Delay(1);
		set_CM_speed(CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM4SpeedPID.output*SPEED_OUTPUT_ATTENUATION);	// hold motor please :)
    //set_CM_speed(2000,2000,2000,2000);
		counter++;
		if(counter>=5000)               //Emergency stop testing.
		{
			CM1SpeedPID.ref = 0;
	    CM2SpeedPID.ref = 0;
	    CM3SpeedPID.ref = 0;
	    CM4SpeedPID.ref = 0;
			
  }
}
}
/* =========================================== */

