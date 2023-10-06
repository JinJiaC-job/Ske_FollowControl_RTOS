#include "ids830can.h"

#include "lkmoto.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "tim.h"

uint8_t ids830_position[8] = {0};//电缸接收数据
uint8_t ids830_currentAndspeed[8] = {0};//电缸接收数据

void IDS830_can_send(uint8_t *buf,uint8_t id)
{
	uint8_t motorId = id;
	hCAN1_TxHeader.StdId = motorId;
	hCAN1_TxHeader.ExtId = 0x00;
	hCAN1_TxHeader.RTR = CAN_RTR_DATA;
	hCAN1_TxHeader.IDE = CAN_ID_STD;
	hCAN1_TxHeader.DLC = 8;
	static uint32_t txMailBox;
	HAL_CAN_AddTxMessage(&hcan, &hCAN1_TxHeader, buf, &txMailBox);
}

void writeData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {group_id, 0x1A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);
	IDS830_can_send(buf, id);
	Delay_ms(Linear_cmd_interval_time);
}

void readData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {group_id, 0x2A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);
	IDS830_can_send(buf, id);
	Delay_ms(Linear_cmd_interval_time);
}

void writeData_oneTomany(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {0x00, 0x8A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);	
	IDS830_can_send(buf, id);
	Delay_ms(Linear_cmd_interval_time);
}

void writeData_oneTomany_CorrNoRes(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 )
{
	uint8_t buf[LEN] = {0x00, 0x9A, reg1, 0x00, 0x00, reg2, 0x00, 0x00};
	buf[4] = *(uint8_t *)(& data1);
	buf[3] = *((uint8_t *)(& data1)+1);
	buf[7] = *((uint8_t *)(& data2));
	buf[6] = *((uint8_t *)(& data2)+1);		
	IDS830_can_send(buf, id);
	Delay_ms(Linear_cmd_interval_time);
}

void LinearActuator_startRun_targetSpeed(uint8_t id, uint16_t targetSpeed)
{
	writeData_pointTopoint(id, 0x00, 0x00, 0x0000, 0x06, targetSpeed);
	readData_pointTopoint(id, 0x00, 0xE8, 0x0000, 0xE9, 0x0000);
}

void LinearActuator_speedmode_runtime(uint8_t id, uint16_t runtime)
{
	writeData_pointTopoint(id, 0x00, 0x02, 0x00c4, 0x0a, runtime);
}

void LinearActuator_startRun_maxspeed_position(uint8_t id, float position, float maxspeed)
{
	position = position * 2500;
	maxspeed = maxspeed*60/4/3000*8192;
	writeData_pointTopoint(id, 0x00, 0x00, 0x0001, 0x1d, maxspeed);
	writeData_pointTopoint(id, 0x00, 0x50, ((int32_t)position & 0xffff0000) >> 16, 0x05, (int32_t)position & 0x0000ffff);
	
	Delay_ms(Linear_cmd_interval_time);
}

void LinearActuator_read_position(uint8_t id)
{
	uint32_t LinAcr_position = 0;
	float LinAcr_position_float = 0;
	
	readData_pointTopoint(id, 0x00, 0xe8, 0x0000, 0xe9, 0x0000);
//	if(CAN_motor_data[2] == 0xe8)
//	{
//		LinAcr_position = ((uint32_t)CAN_motor_data[3]<<24) | ((uint32_t)CAN_motor_data[4]<<16) | ((uint32_t)CAN_motor_data[6]<<8) | ((uint32_t)CAN_motor_data[7]);
		
		*(uint8_t *)(&LinAcr_position) = ids830_position[7];
		*((uint8_t *)(&LinAcr_position)+1) = ids830_position[6];
		*((uint8_t *)(&LinAcr_position)+2) = ids830_position[4];
		*((uint8_t *)(&LinAcr_position)+3) = ids830_position[3];
//	return  LinAcr_position;
    LinAcr_position_float = (int32_t)LinAcr_position/2500.0;
//		HAL_UART_Transmit(&huart1, (uint8_t*)&LinAcr_position, 4, 5);
	  printf("LinearActuator_position%d: %.3f mm\r\n", id, LinAcr_position_float);
//	}
	Delay_ms(Linear_cmd_interval_time);
}

void LinearActuator_read_CurrentandSpeed(uint8_t id)
{
	uint16_t LinAcr_current = 0;
	uint16_t LinAcr_speed = 0;
	
	readData_pointTopoint(id, 0x00, 0xe2, 0x0000, 0xe4, 0x0000);
//	if(CAN_motor_data[2] == 0xe2 && CAN_motor_data[5] == 0xe4)
//	{
		*(uint8_t *)(&LinAcr_current) = ids830_currentAndspeed[4];
		*((uint8_t *)(&LinAcr_current)+1) = ids830_currentAndspeed[3];
		float LinAcr_current_float = (int16_t)LinAcr_current/100.0;
		printf("LinearActuator_current%d: %.3fA\r\n", id, LinAcr_current_float);
//		HAL_UART_Transmit(&huart1, (uint8_t*)&LinAcr_current, 2, 5);
		
		*(uint8_t *)(&LinAcr_speed) = ids830_currentAndspeed[7];
		*((uint8_t *)(&LinAcr_speed)+1) = ids830_currentAndspeed[6];
		float LinAcr_speed_float = (int16_t)LinAcr_speed/8192.0*3000.0;
		printf("LinearActuator_speed%d: %.3frpm\r\n", id, LinAcr_speed_float);
//		HAL_UART_Transmit(&huart1, (uint8_t*)&LinAcr_speed, 2, 5);
//	}
	Delay_ms(Linear_cmd_interval_time);
}




