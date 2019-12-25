/*
 * @Author: your name
 * @Date: 2019-12-18 18:51:14
 * @LastEditTime: 2019-12-24 19:35:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /modbus-slave/modbus-slave.h
 */
#ifndef _MODBUS_SLAVE__H
#define _MODBUS_SLAVE__H

#include "modbusCRC.h"
//宏定义缓冲区长度
#define MAX_SEND_BUFF_LEN 256 //发送缓冲长度
#define MAX_RECV_BUFF_LEN 256 //接收缓冲长度
//地址宏定义
#define RADIO_BROAD_ADDR 0x00 //广播地址
#define SLAVE_ADDR 0x01 //从站地址
//功能码
#define READ_HOLDING_REG 0x03 //读保持寄存器
#define WRITE_SINGLE_REG 0x06  //写单个寄存器
#define WRITE_MULTIPLE_REG 0x10//写多个寄存器
//寄存器定义
#define TEMP_START_ADDR 10000 //温度值寄存器起始地址 10000~10099
#define TEMP_END_ADDR 10099  //温度值寄存器结束地址
#define TEMP_DATA_LEN 100

#define TEMP_HUMI_START_ADDR 10100 //温湿度值寄存器起始地址 10100~10139
#define TEMP_HUMI_END_ADDR 10139 //温湿度值寄存器结束地址 10139
#define TEMPHUMI_DATA_LEN 40

#define WATER_IMMER_START_ADDR 10140 //水浸状态寄存器起始地址 101140 ~10159
#define WATER_IMMER_END_ADDR 10159 //水浸状态寄存器结束地址 101140 ~10159
#define WATER_IMMER_DATA_LEN 20

#define GATE_MAGN_START_ADDR 10160  //门磁状态寄存器起始地址 10160 ~10179
#define GATE_MAGN_END_ADDR 10179   //门磁状态寄存器结束地址 
#define GATE_MAGN_DATA_LEN 20 

#define TEMP_VOLTA_START_ADDR 20000 //温度传感器内部电压寄存器起始地址 20000~20099
#define TEMP_VOLTA_END_ADDR 20099 //温度传感器内部电压寄存器结束地址 
#define TEMP_VOLTA_DATA_LEN 100

#define TEMP_HUMI_VOLTA_START_ADDR 20100 //温湿度传感器内部电压值起始地址 20100~20139
#define TEMP_HUMI_VOLTA_END_ADDR 20139 //温湿度传感器内部电压值结束地址 
#define TEMP_HUMI_VOLTA_DATA_LEN 40

#define WATER_IMMER_VOLTA_START_ADDR 20140 ////水浸传感器内部电压值起始地址 20140~20159
#define WATER_IMMER_VOLTA_END_ADDR 20159 ////水浸传感器内部电压值结束地址 
#define WATER_IMMER_VOLTA_DATA_LEN 20

#define GATE_MAGN_VOLTA_START_ADDR 20160 ////门磁温湿度传感器内部电压值起始地址 20160~20179
#define GATE_MAGN_VOLTA_END_ADDR 20179 ////门磁温湿度传感器内部电压值结束地址
#define GATE_MAGN_VOLTA_DATE_LEN 20

//报文结构体
typedef struct frame_struc
{
    uchar *buf;
    ushort len; 
} FRAME_STRUC;
extern FRAME_STRUC m_Rxd;
extern FRAME_STRUC m_Txd; 
void Data_init();
int Slave_handle_recvFrame(uchar *recv_buff,ushort recv_len,uchar *send_buff);//处理报文
int Slave_ReadHoldingReg_Process(uchar *recv_buff,uchar *send_buff); //读保持寄存器值处理
int Slave_WriteSingleReg_Process(uchar *recv_buff,uchar *send_buff); //写入单个保持寄存器值处理
int Slave_WriteMultipleReg_Process(uchar *recv_buff,uchar *send_buff);//写入多个保持寄存器值处理
int Slave_ErrorHandling(uchar *recv_buff,uchar *send_buff,uchar ErrorType);

#endif