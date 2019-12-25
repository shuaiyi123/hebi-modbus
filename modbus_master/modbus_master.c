/*
 * @Author: your name
 * @Date: 2019-12-18 18:55:03
 * @LastEditTime: 2019-12-24 19:26:00
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /hebi-modbus/modbus_master.c
 */
//----------------------------------------------------------------------------//
//此代码只支持作为Modbus主站设备的Modbus RTU模式
//
//支持的功能码：
//0x03 读保持寄存器（读多个保持寄存器的值）
//0x06 写单个寄存器（写入一个寄存器的值）
//0x10 写多个寄存器（写入多个寄存器的值）
//
//支持的异常码：MAX_SEND_BUFF_LEN
//0x02 非法数据地址（起始地址不在有效范围内）
//0x03 非法数据值（在起始地址的基础上，数量是不合法的）
//----------------------------------------------------------------------------//
#include <stdio.h>

#include "modbus_master.h"
#include "serial_drive.h"
#include "syslog.h"

/* 变量定义 ------------------------------------------------------------------*/
// uchar Modbus_Send_Buff[MAX_SEND_BUFF_LEN]; //发送数据缓冲区
// uchar Modbus_Rcv_Buff[MAX_RECV_BUFF_LEN]; //接收数据缓冲区

FRAME_STRUC m_Txd={NULL,0}; //发送数据结构体
FRAME_STRUC m_Rxd={NULL,0}; //接收数据结构体

//ushort readHold_centRegStartAddr=0; //读保持寄存器当前起始地址

MODBUS_ARGU_STRU modbus_argu={0,0,0,0,0};//保存modbus报文的一些重要参数

ushort temp_value[100]={0};//温度值 100个
ushort tempSen_volt_value[100]={0};//温度传感器内部电压值 100个
ushort humiSen_temp_value[40]={0};//温湿度传感器温度值 40个
ushort humiSen_humi_value[40]={0};//温湿度传感器湿度值 40个
ushort tempHumiSen_volt_value[40]={0}; //温湿度传感器内部电压值 100个
uchar water_immer_value[20]={0};//水浸传感器状态 20个
ushort waterImmer_volt_value[20]={0};//水浸传感器内部电压值 20个
uchar gate_magn_value[20]={0};//门磁传感器状态 20个
ushort gateMagn_volt_value[20]={0};//门磁传感器内部电压值 20个

//----------------------------------------------------------------------------//
//函数功能：Modbus处理报文,帧错误则丢弃
//入口参数：无
//出口参数：无
//最后修改：
//备注：
//----------------------------------------------------------------------------//
int Master_handle_recvFrame(uchar *recv_buff,ushort recv_len,uchar *send_buff)
{
    uchar Modbus_CRC_Rcv_Hi; //接收到的ModbusCRC校验码高字节
    uchar Modbus_CRC_Rcv_Lo; //接收到的ModbusCRC校验码低字节
    ushort Modbus_CRC_Rcv;   //接收到的ModbusCRC校验码
    ushort Modbus_CRC_Cal;   //根据接收到的数据计算出来的CRC值
    
    //----------------------------------------------------------//
    //开始解析
    //----------------------------------------------------------//
    if(recv_buff == NULL ||  send_buff == NULL || recv_len < 5) //如果接收到的一帧的字节数大于4 首先确保帧的长度在正常范围
    {
        logMsg(logErr,"recv_buff is NULL or send_buff is NULL or recv_len lower 5");
        return ERROR;
    }

    Modbus_CRC_Rcv_Lo = recv_buff[recv_len - 2];//接收到的ModbusCRC校验码低字节
    Modbus_CRC_Rcv_Hi = recv_buff[recv_len - 1];//接收到的ModbusCRC校验码高字节
    Modbus_CRC_Rcv = MAKEWORD(Modbus_CRC_Rcv_Lo,Modbus_CRC_Rcv_Hi);//接收到的ModbusCRC校验码（16位）
    Modbus_CRC_Cal = crc16(recv_buff, recv_len - 2);//根据接收到的数据计算CRC值
    
    if(Modbus_CRC_Cal == Modbus_CRC_Rcv)//如果计算的CRC值与接受的CRC值相等
    {
        if(SLAVE_ADDR == recv_buff[0]) //如果是本机地址
        {
            switch(recv_buff[1]) //用switch分支语句来确定功能
            {
            case READ_HOLDING_REG:   //读保持寄存器                                           
                Master_ReadHoldingReg_RespProc(recv_buff); //从站响应读保持寄存器处理
                break;     

            case WRITE_SINGLE_REG://写单个保持寄存器
                Master_WriteSingleReg_RespProc(recv_buff);//从站响应写单个保持寄存器处理
                break; 

            case WRITE_MULTIPLE_REG://写多个保持寄存器
                Master_WriteMultipleReg_RespProc(recv_buff);//从站响应写多个保持寄存器处理
                break;  

            default:
                Master_ErrorHandling(recv_buff);//所有功能码都不符合，则返回功能码错误异常响应报文
                return ERROR;
            }
        }
        else
        {
            logMsg(logErr,"slave address error!");
            return ERROR;
        }   
    }
    else
    {
        logMsg(logErr,"crc16 error!");
        return ERROR;
    } 

    return OK;
}
/**
 * @description: 主站发起读保持寄存器
 * @param {type} 
 * @return: 
 */
int Master_ReadHoldingReg_Request(uchar *send_buff,ushort start_addr,ushort reg_num)
{
    ushort loc_send_len=0;
    ushort mod_crc16;

    if(send_buff == NULL)
    {
        logMsg(logErr,"send_buff pointer is NULL");
        return ERROR;
    }

    send_buff[loc_send_len++] = SLAVE_ADDR;//从站地址
    send_buff[loc_send_len++] = READ_HOLDING_REG;//功能码
    
    send_buff[loc_send_len++] = HIBYTE(start_addr);//起始地址高8位
    send_buff[loc_send_len++] = LOBYTE(start_addr);//起始地址低8位
    modbus_argu.readHoldRegStartAddr_03 = start_addr; //起始地址
    
    send_buff[loc_send_len++] = HIBYTE(reg_num);//数据个数
    send_buff[loc_send_len++] = LOBYTE(reg_num);
    modbus_argu.readHoldRegCnt_03 = reg_num;

    mod_crc16 = crc16(send_buff,loc_send_len);
    send_buff[loc_send_len++] = LOBYTE(mod_crc16);//crc16低8位
    send_buff[loc_send_len++] = HIBYTE(mod_crc16);//crc16高8位

    //*send_len = loc_send_len;//保存报文长度

    //******************************
     TransData(send_buff,loc_send_len);
    //******************************
    return OK;
}
/**
 * @description: 主站发起写单个寄存器
 * @param {type} 
 * @return: 
 */
int Master_WriteSingleReg_Request(uchar *send_buff,ushort start_addr,ushort reg_value)
{
    ushort loc_send_len=0;
    ushort mod_crc16;

    if(send_buff == NULL)
    {
        logMsg(logErr,"send_buff pointer is NULL ");
        return ERROR;
    }

    send_buff[loc_send_len++] = SLAVE_ADDR;//从站地址
    send_buff[loc_send_len++] = READ_HOLDING_REG;//功能码

    send_buff[loc_send_len++] = HIBYTE(start_addr);//起始地址高8位
    send_buff[loc_send_len++] = LOBYTE(start_addr);//起始地址低8位

    modbus_argu.writeSingleRegStartAddr_06 = start_addr;
    modbus_argu.writeSingleRegVal_06 = reg_value;

    send_buff[loc_send_len++] = HIBYTE(reg_value);//写值高高8位
    send_buff[loc_send_len++] = LOBYTE(reg_value);//写值低8位

    mod_crc16 = crc16(send_buff,loc_send_len);
    send_buff[loc_send_len++] = LOBYTE(mod_crc16);//crc16低8位
    send_buff[loc_send_len++] = HIBYTE(mod_crc16);//crc16高8位

   // *send_len = loc_send_len;//保存报文长度

    //******************************
    TransData(send_buff,loc_send_len);
    //******************************
    
    return OK;
}
/**
 * @description: 主站发起写多个寄存器
 * @param {start_addr:起始地址,reg_num:寄存器个数,byte_cnt:数据字节数,reg_value:寄存器值首地址} 
 * @return: 
 */
int Master_WriteMultipleReg_Request(uchar *send_buff,ushort start_addr,ushort reg_num,uchar byte_cnt,ushort *reg_value)
{
    ushort loc_send_len=0;
    ushort mod_crc16;
    
    if(send_buff == NULL || reg_value == NULL)
    {
        logMsg(logErr,"send_buff pointer is NULL or reg_value pointer is NULL");
        return ERROR;
    }

    send_buff[loc_send_len++] = SLAVE_ADDR;//从站地址
    send_buff[loc_send_len++] = READ_HOLDING_REG;//功能码

    send_buff[loc_send_len++] = HIBYTE(start_addr);//起始地址高8位
    send_buff[loc_send_len++] = LOBYTE(start_addr);//起始地址低8位
    
    modbus_argu.writeMultRegStartAddr_16 = start_addr;
    modbus_argu.writeMulitRegCnt_16 = reg_num;

    send_buff[loc_send_len++] = HIBYTE(reg_num);//寄存器个数高位
    send_buff[loc_send_len++] = HIBYTE(reg_num);//寄存器个数低位

    send_buff[loc_send_len++] = byte_cnt;//写数据字节数

    for(uchar i=0;i<reg_num;i++)
    {
        send_buff[loc_send_len++] = HIBYTE(reg_value[i]);//数值高位
        send_buff[loc_send_len++] = LOBYTE(reg_value[i]);//数值低位
    } 

    mod_crc16 = crc16(send_buff,loc_send_len);
    send_buff[loc_send_len++] = LOBYTE(mod_crc16);//crc16低8位
    send_buff[loc_send_len++] = HIBYTE(mod_crc16);//crc16高8位

    //*send_len = loc_send_len;//保存报文长度

    //******************************
    TransData(send_buff,loc_send_len);
    //******************************
    
    return OK;
}
/**
 * @description: //从站响应读保持寄存器处理
 * @param {type} 
 * @return: 
 */
int Master_ReadHoldingReg_RespProc(uchar *recv_buff)//从站响应读保持寄存器处理
{
    ushort start_addr_reg; //读取的寄存器起始地址
    uchar byte_cnt; //数据字节数量
    ushort i, j;     //临时变量

    if(recv_buff == NULL)
    {
        logMsg(logErr,"recv_buff pointer is NULL");
        return ERROR;
    }

    byte_cnt = recv_buff[2];//数据字节数
    //寄存器起始地址在温度传感器温度值的寄存器正确范围内
    if(modbus_argu.readHoldRegStartAddr_03 >= TEMP_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < TEMP_END_ADDR) 
    {      
        if(byte_cnt != 2*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error 01!");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -TEMP_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 2)//读取寄存器的数据
        {
            temp_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
        }
    } 
    //寄存器起始地址在温湿度传感器温湿度值的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= TEMP_HUMI_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < TEMP_HUMI_END_ADDR)
    {
        if(byte_cnt != 4*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error 02!");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -TEMP_HUMI_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 4)//读取寄存器的数据
        {
            humiSen_temp_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
            humiSen_humi_value[i] = MAKEWORD(recv_buff[j+3],recv_buff[j+2]);

        }
    }
    //寄存器起始地址在水浸传感器水浸状态的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= WATER_IMMER_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < WATER_IMMER_END_ADDR)
    {
        if(byte_cnt != modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error 03!");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -WATER_IMMER_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 1)//读取寄存器的数据
        {
            water_immer_value[i] = recv_buff[j];
        }
    }
    //寄存器起始地址在门磁传感器门磁状态的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= GATE_MAGN_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < GATE_MAGN_END_ADDR)
    {
        if(byte_cnt != modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error 04!");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -GATE_MAGN_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 1)//读取寄存器的数据
        {
            gate_magn_value[i] = recv_buff[j];
        }
    }
    //寄存器起始地址在温度传感器内部电的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= TEMP_VOLTA_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < TEMP_VOLTA_END_ADDR)
    {
        if(byte_cnt != 2*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error! 05");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -TEMP_VOLTA_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 2)//读取寄存器的数据
        {
            tempSen_volt_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
        }
    }
    //寄存器起始地址在温湿度传感器内部电压值的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= TEMP_HUMI_VOLTA_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < TEMP_HUMI_VOLTA_END_ADDR)
    {
        if(byte_cnt != 2*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error! 06");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -TEMP_HUMI_VOLTA_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 2)//读取寄存器的数据
        {
            tempHumiSen_volt_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
        }
    }
    //寄存器起始地址在水浸传感器内部电压值的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= WATER_IMMER_VOLTA_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < WATER_IMMER_VOLTA_END_ADDR)
    {
        if(byte_cnt != 2*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error! 07");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -WATER_IMMER_VOLTA_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 2)//读取寄存器的数据
        {
            waterImmer_volt_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
        }
    }
    //寄存器起始地址在门磁传感器内部电压值的寄存器正确范围内
    else if(modbus_argu.readHoldRegStartAddr_03  >= GATE_MAGN_VOLTA_START_ADDR && modbus_argu.readHoldRegStartAddr_03  < GATE_MAGN_VOLTA_END_ADDR)
    {
        if(byte_cnt != 2*modbus_argu.readHoldRegCnt_03)
        {
            logMsg(logErr,"number of reading hold register error! 08");
            return ERROR;
        }
        
        start_addr_reg = modbus_argu.readHoldRegStartAddr_03 -GATE_MAGN_VOLTA_START_ADDR;
        for(i = start_addr_reg, j = 3; i < start_addr_reg + modbus_argu.readHoldRegCnt_03; i++, j += 2)//读取寄存器的数据
        {
            gateMagn_volt_value[i] = MAKEWORD(recv_buff[j+1],recv_buff[j]);
        }
    }
    else
    {
        logMsg(logErr,"invalid register start address!");
        return ERROR;
    }
    
    return OK;
}
/**
 * @description: 
 * @param {type} 
 * @return: 
 */
int Master_WriteSingleReg_RespProc(uchar *recv_buff)//从站响应写单个保持寄存器处理
{
    ushort start_addr;
    ushort reg_val;

    if(recv_buff == NULL)
    {
        logMsg(logErr,"recv_buff pointer is NULL or send_buff pointer is NULL");
        return ERROR;
    }

    start_addr = MAKEWORD(recv_buff[3],recv_buff[2]);
    reg_val = MAKEWORD(recv_buff[5],recv_buff[4]);

    if(modbus_argu.writeSingleRegStartAddr_06 != start_addr)
    {
        logMsg(logInfo,"start address of single register from slave respose is error!!");
        return ERROR;
    }
    
    if(modbus_argu.writeSingleRegVal_06 != reg_val)
    {
        logMsg(logInfo,"data value of single register from slave respose is error!!");
        return ERROR;
    }
    
    logMsg(logInfo,"writting value of single register  succeeded!");

    return OK;
}
/**
 * @description: 
 * @param {type} 
 * @return: 
 */
int Master_WriteMultipleReg_RespProc(uchar *recv_buff)//从站响应写多个保持寄存器处理
{
    ushort start_addr;
    ushort reg_num;

    if(recv_buff == NULL)
    {
        logMsg(logErr,"recv_buff pointer is NULL or send_buff pointer is NULL");
        return ERROR;
    }

    start_addr = MAKEWORD(recv_buff[3],recv_buff[2]);
    reg_num = MAKEWORD(recv_buff[5],recv_buff[4]);

    if(modbus_argu.writeSingleRegStartAddr_06 != start_addr)
    {
        logMsg(logInfo,"start address of multiply register from slave respose is error!!");
        return ERROR;
    }

    if(modbus_argu.writeMulitRegCnt_16 != reg_num)
    {
        logMsg(logInfo,"data value of multiply register from slave respose is error!!");
        return ERROR;
    }
    
    logMsg(logInfo,"writting value of multiply holding register succeeded!");

    return OK;
}
/**
 * @description: 错误码处理
 * @param {type} 
 * @return: 
 */
void Master_ErrorHandling(uchar *recv_buff)
{
    uchar invalid_func; //异常功能码
    uchar excep_code; //异常码

    invalid_func = recv_buff[1];
    excep_code = recv_buff[2];

    logMsg(logErr,"invalid function = %02x",invalid_func);

    switch(invalid_func)
    {
    case 0x01:
        logMsg(logErr,"unusual function code!");
        break;
    case 0x02:
        logMsg(logErr,"unusual register address!");
        break;

    case 0x03:
        logMsg(logErr,"unusual data value!");
        break;
    default: 
        logMsg(logErr,"Unknown exception code,exception code = %02x",excep_code);
    }
}