#ifndef IDS830CAN_H
#define IDS830CAN_H

#include "can.h"
#include "stdio.h"
#include "lkmoto.h"

//电缸输入信号int32_t position：50000<-->20mm; 1mm<-->2500

//控制命令间隔
#define Linear_cmd_interval_time 2

extern uint8_t ids830_position[8];//电缸接收数据
extern uint8_t ids830_currentAndspeed[8];//电缸接收数据

void IDS830_can_send(uint8_t *buf,uint8_t id);

/*点对点的写数据操作，掉电不保存。主机发送数据指令，接收正确后，从机返回相应
数据指令。例如，主机发送指令为：0x05 0x00 0x1A 0x06 0x00 0x08 0x00 0x00 0x01 具体对
应的指令内容为，对 ID 号为 0x05，组号为 0 的从机发送了速度指令（0x06 0x00 0x08 ）为
8，并启动电机（0x00 0x00 0x01）的命令。其中 0x1A 是指令的功能码，表示写数据，但不
保存数据。从机接收到数据后数据即时生效。如果寄存器地址设置为 0xFF,从机则自动识别
该指令为空指令，不执行任何操作。如果主机只操作单一寄存器时，另一寄存器地址请设置为 0xFF*/

//发送指令格式如下：
//接收正确后返回指令格式如下：功能码：0X1B
//接收数据出错后返回指令格式如下：功能码：0X1C
void writeData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 );


/*点对点的读数据操作，主机发送数据指令，接收正确后，从机返回相应寄存器地址数
据内容。例如，主机发送指令为：0x05 0x00 0x2A 0xE8 0x00 0x00 0xE9 0x00 0x00 具体对应
的指令内容为，对 ID 号为 0x05 的从机发送读位置反馈高 16 位指令（0xE8 0x00 0x00 ），
位置反馈低 16 位指令（0xE9 0x00 0x00 ）。其中 0x2A 是指令的功能码，表示读数据，从机
接收到指令后，把地址相应的数据内容上传，功能码变为 0x2B。如果寄存器地址设置为 0xFF, 从机则自动识别该指令为空指令，不执行任何操作。如果主机只操作单一寄存器时，另一寄
存器地址请设置为 0xFF。*/

//发送指令格式如下：
//接收正确后返回指令格式如下：功能码：0X2B
//接收数据出错后返回指令格式如下：功能码：0X2C
void readData_pointTopoint(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 );


/*一对多的写数据操作，掉电不保存。主机发送数据指令，接收正确后，从机返回相应
数据指令。例如，主机发送指令为：0x00 0x01 0x8A 0x06 0x00 0x08 0x00 0x00 0x01 具体对
应的指令内容为，对全局的组号为 0x01 的所有从机发送了速度指令（0x06 0x00 0x08 ）为 8，
并启动电机（0x00 0x00 0x01）的命令。其中 0x8A 是指令的功能码，表示写数据，但不保存
数据。从机接收到数据后数据即时生效。如果寄存器地址设置为 0xFF, 从机则自动识别该指
令为空指令，不执行任何操作。如果主机只操作单一寄存器时，另一寄存器地址请设置为0xFF。*/

//发送指令格式如下：
//接收正确后返回指令格式如下：功能码：0X8B
//接收数据出错后返回指令格式如下：功能码：0X8C
void writeData_oneTomany(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 );

/*一对多的写数据操作，掉电不保存。主机发送数据指令，接收正确后，从机不返回相
应数据指令。但接收数据有误则返回报错帧。例如，主机发送指令为：0x00 0x01 0x8A 0x06
0x00 0x08 0x00 0x00 0x01 具体对应的指令内容为，对全局的组号为 0x01 的所有从机发送
了速度指令（0x06 0x00 0x08 ）为 8，并启动电机（0x00 0x00 0x01）的命令。其中 0x9A 是
指令的功能码，表示写数据，但不保存数据。从机接收到数据后数据即时生效。如果寄存器
地址设置为 0xFF,从机则自动识别该指令为空指令，不执行任何操作。如果主机只操作单一
寄存器时，另一寄存器地址请设置为 0xFF。*/

//发送指令格式如下：
//接收数据出错后返回指令格式如下：功能码：0X9C
void writeData_oneTomany_CorrNoRes(uint8_t id, uint8_t group_id, uint8_t reg1, uint16_t data1, uint8_t reg2, uint16_t data2 );


//参数寄存器地址映射列表：
#define servomoto_enable 0x00  //写入电机使能
#define control_mode     0x02  //控制模式给定命令来源选择
#define PCmode_velocity  0x06  //PC 模式-速度给定,实际电机转速=(写入值/8192)*3000
#define CAN_group        0x0b  //CAN-组号,设置一对多模式时，从机接收组
#define CAN_reporttime   0x0c  //CAN-报告时间,设置自动向主机报告状态信息时间间隔，大于 0 时起作用,单位：ms
#define CANID_identify   0x0d  //从机 CAN-ID号,从机自身识别号

/*
说明 Byte0 Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 Byte7
读写  组号 功能码 地址1 高8位 低8  地址2 高8位 低8位
写操作 00   1A    00    00   01    06   00     00
返回   00   1B    00    00   01    06   00     00
写操作地址 0，启动指令。地址 06，写入目标速度为 0，接收正确后原数据返回。
读操作 00   2A    E8    00   00    E9   00     00
返回   00   2B    E8    00   10    E9   02     00
读操作，读地址位置反馈高 16 位 E8，低 16 位 E9。相应返回对应实时位置*/
void LinearActuator_startRun_targetSpeed(uint8_t id, uint16_t targetSpeed);


/*
速度模式可预设操作寄存器
组号 功能码 地址1 高8位 低8  地址2 高8位 低8位
00   1A     02    00   C4    0a    01   01
00   1B     02    00   C4    0a    01   01
地址 1 设置为速度模式，地址 2 设置加减速时间为 1.接收正确后原数据返回*/
void LinearActuator_speedmode_runtime(uint8_t id, uint16_t runtime);


/*
位置模式必要操作寄存器
组号 功能码 地址1 高8位 低8  地址2 高8位 低8位
00    1A    00    00   01    1d    20   00
00    1B    00    00   01    1d    20   00
地址 1 发送启动指令，地址 2 设置最高速度 3000RPM，接收正确后原数据返回
00    1A    50    00   00    05    00   00
00    1B    50    00   00    05    00   00
发送目标位置，地址 1 位置高 16 位，地址 2 位置低 16 位。接收正确原数据返回*/
void LinearActuator_startRun_maxspeed_position(uint8_t id, float position, float maxspeed);

/*
电缸读取位置反馈信息
*/
void LinearActuator_read_position(uint8_t id);

/*
电缸读取输出电流和转速
读取电流功能码：0xe2  实际电流要缩小100倍
读取转速功能码：0xe4  返回的数字量16384对应实际转速6000rpm
*/
void LinearActuator_read_CurrentandSpeed(uint8_t id);


#endif

