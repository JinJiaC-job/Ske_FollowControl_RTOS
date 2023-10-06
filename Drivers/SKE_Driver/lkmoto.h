// LK 电机驱动文件

#ifndef __LKMOTO_H
#define __LKMOTO_H



#include "stm32f1xx_hal.h"
#include "can.h"

//CAN_TxHeaderTypeDef canTxHeader;
//CAN_RxHeaderTypeDef canRxHeader;


// LK 电机驱动文件

#include "lkmoto.h"

// data 长度 DLC
#define LEN 8

//控制命令间隔
#define command_interval_time 2

extern uint8_t CAN_motor_data[8];//电机接收数据
extern uint8_t CAN_motor_angle[8];
extern uint8_t CAN_motor_currentAndspeed[8];
extern uint32_t circleAngle;//电机角度值

//can 总线 send 函数 移植仅需修改此函数
// id 0x140 + (1~32)


void can_send(uint8_t *buf,uint8_t id);
void can_msg_process(uint8_t id,uint8_t *buf);
void ms_Delay(uint16_t t_ms);


//motor1:角度需要乘上减速比：360°=36000*10=360000
//motor2:角度需要乘上减速比：360°=36000*8=288000
//motor3:角度需要乘上减速比：360°=36000*36=1296000
//motor4:角度需要乘上减速比：360°=36000*8=288000
//motor5:角度需要乘上减速比：360°=36000*36=1296000






// 发送
//  读取 PID 参数命令 0x30
// 主机发送该命令读取当前电机的的 PID 参数
// 数据域 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//回复
// 驱动回复数据中包含了各个控制环路的 PI 参数。
// 数据域 说明 数据
// DATA[0] 命令字节 0x30
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

void read_pid(uint8_t id);

//  写入 PID 参数到 RAM 命令 0x31
// DATA[0] 命令字节 0x31
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_pid(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);

//  写入 PID 参数到 ROM 命令 0x32
// DATA[0] 命令字节 0x32
// DATA[1] NULL 0x00
// DATA[2] 位置环 P 参数 DATA[2] = anglePidKp
// DATA[3] 位置环 I 参数 DATA[3] = anglePidKi
// DATA[4] 速度环 P 参数 DATA[4] = speedPidKp
// DATA[5] 速度环 I 参数 DATA[5] = speedPidKi
// DATA[6] 转矩环 P 参数 DATA[6] = iqPidKp
// DATA[7] 转矩环 I 参数 DATA[7] = iqPidKi

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致
void write_pid_to_rom(uint8_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);





//  读取加速度命令 0x33
// 数据域 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复数据中包含了加速度参数。加速度数据 Accel 为 int32_t 类型，单位 1dps/s
// 数据域 说明 数据
// DATA[0] 命令字节 0x33
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字节 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

void read_acc(uint8_t id);




//  写入加速度到 RAM 命令 0x34
// DATA[0] 命令字节 0x34
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 加速度低字节 1 DATA[4] = *(uint8_t *)(&Accel)
// DATA[5] 加速度字节 2 DATA[5] = *((uint8_t *)(&Accel)+1)
// DATA[6] 加速度字节 3 DATA[6] = *((uint8_t *)(&Accel)+2)
// DATA[7] 加速度字节 4 DATA[7] = *((uint8_t *)(&Accel)+3)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_acc(uint8_t id, float Accel);




//  读取编码器命令 0x90
// 数据域 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 编码器位置 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383），为编码器原始位置减
// 去编码器零偏后的值。
// 2. 编码器原始位置 encoderRaw（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 3. 编码器零偏 encoderOffset（uint16_t 类型，14bit 编码器的数值范围 0~16383），该点作为电机角
// 度的 0 点。
// 数据域 说明 数据
// DATA[0] 命令字节 0x90
// DATA[1] NULL 0x00
// DATA[2] 编码器位置低字节 DATA[2] = *(uint8_t *)(&encoder)
// DATA[3] 编码器位置高字节 DATA[3] = *((uint8_t *)(&encoder)+1)
// DATA[4] 编码器原始位置低字节 DATA[4] = *(uint8_t *)(&encoderRaw)
// DATA[5] 编码器原始位置高字节 DATA[5] = *((uint8_t *)(&encoderRaw)+1)
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

void read_encoder(uint8_t id);






//  写入编码器值到 ROM 作为电机零点命令 0x91
// 主机发送该命令以设置编码器的零偏，其中，需要写入的编码器值 encoderOffset 为 uint16_t 类型，
// 14bit 编码器的数值范围 0~16383。
// 数据域 说明 数据
// DATA[0] 命令字节 0x91
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] 编码器零偏低字节 DATA[6] = *(uint8_t *)(&encoderOffset)
// DATA[7] 编码器零偏高字节 DATA[7] = *((uint8_t *)(&encoderOffset)+1)

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_encoder_offset(uint8_t id, uint16_t encoderOffset);



//  写入当前位置到 ROM 作为电机零点命令 0x19
// 数据域 说明 数据
// DATA[0] 命令字节 0x19
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void write_current_position_to_rom(uint8_t id);



//  读取多圈角度命令 0x92
// 主机发送该命令以读取当前电机的多圈绝对角度值
// 数据域 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机角度 motorAngle，为 int64_t 类型数据，正值表示顺时针累计角度，负值表示逆时针累计角
// 度，单位 0.01°/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0x92
// DATA[1] 角度低字节 1 DATA[1] = *(uint8_t *)(&motorAngle)
// DATA[2] 角度字节 2 DATA[2] = *((uint8_t *)(& motorAngle)+1)
// DATA[3] 角度字节 3 DATA[3] = *((uint8_t *)(& motorAngle)+2)
// DATA[4] 角度字节 4 DATA[4] = *((uint8_t *)(& motorAngle)+3)
// DATA[5] 角度字节 5 DATA[5] = *((uint8_t *)(& motorAngle)+4)
// DATA[6] 角度字节 6 DATA[6] = *((uint8_t *)(& motorAngle)+5)
// DATA[7] 角度字节 7 DATA[7] = *((uint8_t *)(& motorAngle)+6)



void read_angle(uint8_t id);



//  读取单圈角度命令 0x94
// 数据域 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机单圈角度 circleAngle，为 uint16_t 类型数据，以编码器零点为起始点，顺时针增加，再次到
// 达零点时数值回 0，单位 0.01°/LSB，数值范围 0~35999。
// 数据域 说明 数据
// DATA[0] 命令字节 0x94
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] 单圈角度低字节 DATA[6] = *(uint8_t *)(& circleAngle)
// DATA[7] 单圈角度高字节 DATA[7] = *((uint8_t *)(& circleAngle)+1)

void read_angle_single(uint8_t id);



//  清除电机角度命令（设置电机初始位置） 0x95 厂家暂未实现
// 回复 电机在收到命令后回复主机，回复命令和接收命令一致


//  读取电机状态 1 和错误标志命令 0x9A
// 该命令读取当前电机的温度、电压和错误状态标志
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数：
// 1. 电机温度 temperature（int8_t 类型，单位 1℃/LSB）。
// 2. 电压 voltage（uint16_t 类型，单位 0.1V/LSB）。
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字节 DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字节 DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字节 DATA[7]=errorState
// 备注：
// 1. errorState 各个位具体状态表如下
// errorState 位 状态说明 0 1
// 0 电压状态 电压正常 低压保护
// 1 无效
// 2 无效
// 3 温度状态 温度正常 过温保护
// 4 无效
// 5 无效
// 6 无效
// 7 无效


void read_status(uint8_t id);

//  清除电机错误标志命令 0x9B

// 数据域 说明 数据
// DATA[0] 命令字节 0x9B
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

//驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下参数：
// 1. 电机温度 temperature（int8_t 类型，单位 1℃/LSB）。
// 2. 电压 voltage（uint16_t 类型，单位 0.1V/LSB）。
// 3. 错误标志 errorState（为 uint8_t 类型，各个位代表不同的电机状态）。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9A
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] NULL 0x00
// DATA[3] 电压低字节 DATA[3] = *(uint8_t *)(&voltage)
// DATA[4] 电压高字节 DATA[4] = *((uint8_t *)(& voltage)+1)
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] 错误状态字节 DATA[7]=errorState
// 备注：
// 1. 电机状态没有恢复正常时，错误标志无法清除。
// 2. errorState 各个位具体状态参考读取电机状态 1 和错误标志命令。

void clear_error(uint8_t id);

//  读取电机状态 2 命令 0x9C


// 该命令读取当前电机的温度、电压、转速、编码器位置。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9C
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void read_status2(uint8_t id);



//  读取电机状态 3 命令 0x9D
// 该命令读取当前电机的温度和相电流数据
// 数据域 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据包含了以下数据：
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）
// 2. A 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 3. B 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 4. C 相电流数据，数据类型为 int16_t 类型，对应实际相电流为 1A/64LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0x9D
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] A 相电流低字节 DATA[2] = *(uint8_t *)(&iA)
// DATA[3] A 相电流高字节 DATA[3] = *((uint8_t *)(& iA)+1)
// DATA[4] B 相电流低字节 DATA[4] = *(uint8_t *)(&iB)
// DATA[5] B 相电流高字节 DATA[5] = *((uint8_t *)(& iB)+1)
// DATA[6] C 相电流低字节 DATA[6] = *(uint8_t *)(&iC)
// DATA[7] C 相电流高字节 DATA[7] = *((uint8_t *)(& iC)+1)


void read_status3(uint8_t id);






//  电机关闭命令 0x80

// 关闭电机，同时清除电机运行状态和之前接收的控制指令
// 数据域 说明 数据
// DATA[0] 命令字节 0x80
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_close(uint8_t id);


//  电机停止命令 0x81
// 停止电机，但不清除电机运行状态和之前接收的控制指令
// 数据域 说明 数据
// DATA[0] 命令字节 0x81
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_stop(uint8_t id);


//  电机运行命令 0x88
// 从电机停止命令中恢复电机运行（恢复停止前的控制方式）
// 数据域 说明 数据
// DATA[0] 命令字节 0x88
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] NULL 0x00
// DATA[5] NULL 0x00
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 回复 电机在收到命令后回复主机，回复命令和接收命令一致

void motor_run(uint8_t id);


//  转矩开环控制命令 0xA0
// 主机发送该命令以控制电机的开环输出功率，控制值 powerControl 为 int16_t 类型，数值范围-1000~
// 1000，（电机母线电流和扭矩因不同电机而异）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 输出功率控制值低字节 DATA[4] = *(uint8_t *)(& powerControl)
// DATA[5] 输出功率控制值高字节 DATA[5] = *((uint8_t *)(& powerControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 1. 该命令中的控制值 powerControl 不受上位机(LK-Motor Tool)中的 Max Power 值限制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的输出功率值（int16_t 类型，范围-1000~1000）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA0
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 输出功率值低字节 DATA[2] = *(uint8_t *)(& power)
// DATA[3] 输出功率值高字节 DATA[3] = *((uint8_t *)(&power)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void torque_open_loop(uint8_t id, int16_t powerControl);



//  转矩闭环控制命令 0xA1
// 主机发送该命令以控制电机的转矩电流输出，控制值 iqControl 为 int16_t 类型，数值范围-2000~ 2000，
// 对应实际转矩电流范围-32A~32A（母线电流和电机的实际扭矩因不同电机而异）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 转矩电流控制值低字节 DATA[4] = *(uint8_t *)(&iqControl)
// DATA[5] 转矩电流控制值高字节 DATA[5] = *((uint8_t *)(&iqControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA1
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void torque_close_loop(uint8_t id, int16_t iqControl);



//  速度闭环控制命令 0xA2
// 主机发送该命令以控制电机的速度，控制值 speedControl 为 int32_t 类型，对应实际转速为 0.01dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 速度控制低字节 DATA[4] = *(uint8_t *)(&speedControl)
// DATA[5] 速度控制 DATA[5] = *((uint8_t *)(&speedControl)+1)
// DATA[6] 速度控制 DATA[6] = *((uint8_t *)(&speedControl)+2)
// DATA[7] 速度控制高字节 DATA[7] = *((uint8_t *)(&speedControl)+3)

// 1. 该命令下电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA2
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void speed_close_loop(uint8_t id, int32_t speedControl);




//  位置闭环控制命令 1 0xA3
// 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下的控制值 angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制。
// 2. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 3. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 4. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA3
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop(uint8_t id, int32_t angleControl);



//  位置闭环控制命令 2 0xA4
// 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int32_t 类型，对应实际位
// 置为 0.01degree/LSB，即 36000 代表 360°，电机转动方向由目标位置和当前位置的差值决定。
// 控制值 maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下的控制值 angleControl 受上位机(LK-Motor Tool)中的 Max Angle 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA4
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)



void angle_close_loop_with_speed(uint8_t id, float angleControl, float maxSpeed);


//  位置闭环控制命令 3 0xA5
// 主机发送该命令以控制电机的位置（单圈角度）， 控制值 angleControl 为 uint16_t 类型，数值范围
// 0~35999，对应实际位置为 0.01degree/LSB，即实际角度范围 0°~359.99°。
// 控制值 spinDirection 设置电机转动的方向，为 uint8_t 类型，0x00 代表顺时针，0x01 代表逆时针。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字节 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注：
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA5
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction(uint8_t id, int32_t angleControl, uint8_t spinDirection);


//  位置闭环控制命令 4 0xA6
// 主机发送该命令以控制电机的位置（单圈角度）。
// 1. 角度控制值 angleControl 为 uint16_t 类型，数值范围 0~35999，对应实际位置为 0.01degree/LSB，
// 即实际角度范围 0°~359.99°。
// 2. spinDirection 设置电机转动的方向，为 uint8_t 类型，0x00 代表顺时针，0x01 代表逆时针。
// 3. maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 转动方向字节 DATA[1] = spinDirection
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制高字节 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] NULL 0x00
// DATA[7] NULL 0x00
// 备注：
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 2. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA6
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_speed(uint8_t id, int32_t angleControl, uint8_t spinDirection, uint16_t maxSpeed);


//  位置闭环控制命令 5 0xA7
// 主机发送该命令以控制电机的位置增量， 控制值 angleControl 为 int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] NULL 0x00
// DATA[2] NULL 0x00
// DATA[3] NULL 0x00
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该命令下电机的最大速度由上位机(LK-Motor Tool)中的 Max Speed 值限制。
// 2. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 3. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA7
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop4(uint8_t id, int32_t angleControl);





//  位置闭环控制命令 6 0xA8
// 主机发送该命令以控制电机的位置增量， 控制值 angleControl 为 int32_t 类型，对应实际位置为
// 0.01degree/LSB。电机的转动方向由控制量的符号确定。
// 控制值 maxSpeed 限制了电机转动的最大速度，为 uint16_t 类型，对应实际转速 1dps/LSB。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] NULL 0x00
// DATA[2] 速度限制低字节 DATA[2] = *(uint8_t *)(&maxSpeed)
// DATA[3] 速度限制高字节 DATA[3] = *((uint8_t *)(&maxSpeed)+1)
// DATA[4] 位置控制低字节 DATA[4] = *(uint8_t *)(&angleControl)
// DATA[5] 位置控制 DATA[5] = *((uint8_t *)(&angleControl)+1)
// DATA[6] 位置控制 DATA[6] = *((uint8_t *)(&angleControl)+2)
// DATA[7] 位置控制高字节 DATA[7] = *((uint8_t *)(&angleControl)+3)
// 备注：
// 1. 该控制模式下，电机的最大加速度由上位机(LK-Motor Tool)中的 Max Acceleration 值限制。
// 2. 该控制模式下，电机的最大转矩电流由上位机(LK-Motor Tool)中的 Max Torque Current 值限
// 制。

// 驱动回复（1 帧）
// 电机在收到命令后回复主机，该帧数据中包含了以下参数。
// 1. 电机温度 temperature（int8_t 类型，1℃/LSB）。
// 2. 电机的转矩电流值 iq（int16_t 类型，范围-2048~2048，对应实际转矩电流范围-33A~33A）。
// 3. 电机转速 speed（int16_t 类型，1dps/LSB）。
// 4. 编码器位置值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383）。
// 数据域 说明 数据
// DATA[0] 命令字节 0xA8
// DATA[1] 电机温度 DATA[1] = *(uint8_t *)(&temperature)
// DATA[2] 转矩电流低字节 DATA[2] = *(uint8_t *)(&iq)
// DATA[3] 转矩电流高字节 DATA[3] = *((uint8_t *)(&iq)+1)
// DATA[4] 电机速度低字节 DATA[4] = *(uint8_t *)(&speed)
// DATA[5] 电机速度高字节 DATA[5] = *((uint8_t *)(&speed)+1)
// DATA[6] 编码器位置低字节 DATA[6] = *(uint8_t *)(&encoder)
// DATA[7] 编码器位置高字节 DATA[7] = *((uint8_t *)(&encoder)+1)

void angle_close_loop_with_direction_and_angle_and_max_speed(uint8_t id, int32_t angleControl, uint16_t maxSpeed);

void Ske_pose_1(void);
void Ske_pose_2(void);
void Ske_pose_3(void);
void Ske_pose_4(void);
void Ske_pose_5(void);
void Ske_pose_6(void);

void read_5torque_current(void);
void ske_base_position(void);



























#endif














