/*
 * @Author: your name
 * @Date: 2019-12-18 18:14:25
 * @LastEditTime: 2019-12-18 18:38:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /modbus-slave/crc16.h
 */
#ifndef _MODBUSCRC__H
#define _MODBUSCRC__H

#include "gtypedef.h"
ushort crc16(uchar *buf, uint dataLen);//查找法(效率高)
ushort Modbus_CRC16(uchar *puchMsg, uint dataLen);//计算法
#endif