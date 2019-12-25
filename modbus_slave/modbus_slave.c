/*
 * @Author: your name
 * @Date: 2019-12-18 18:50:50
 * @LastEditTime: 2019-12-24 19:41:51
 * @LastEditors: Please set LastEditors
 * @Description:Modbus从站设备的Modbus RTU模式
 * @FilePath: /modbus-slave/modbus_slave.c
 */
//----------------------------------------------------------------------------//
//此代码只支持作为Modbus从站设备的Modbus RTU模式
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

#include "modbus_slave.h"
#include "serial_drive.h"
#include "syslog.h"

/* 变量定义 ------------------------------------------------------------------*/
//uchar Modbus_Send_Buff[MAX_SEND_BUFF_LEN]; //发送数据缓冲区
//uchar Modbus_Rcv_Buff[MAX_RECV_BUFF_LEN]; //接收数据缓冲区

FRAME_STRUC  m_Txd={NULL,0}; //发送数据结构体
FRAME_STRUC  m_Rxd={NULL,0}; //接收数据结构体

ushort temp_value[100]={0x1234,0x3456};//温度值 100个
ushort tempSen_volt_value[100]={0x1234,0x3456};//温度传感器内部电压值 100个
ushort humiSen_temp_value[40]={0x1234,0x3456};//温湿度传感器温度值 40个
ushort humiSen_humi_value[40]={0x1234,0x3456};//温湿度传感器湿度值 40个
ushort tempHumiSen_volt_value[40]={0x1234,0x3456}; //温湿度传感器内部电压值 100个
uchar water_immer_value[20]={1,1,1,1};//水浸传感器状态 20个
ushort waterImmer_volt_value[20]={0x1234,0x3456};//水浸传感器内部电压值 20个
uchar gate_magn_value[20]={1,1,1,1,1};//门磁传感器状态 20个
ushort gateMagn_volt_value[20]={0x1234,0x3456};//门磁传感器内部电压值 20个

//数据初始化
void Data_init()
{
    temp_value[99] = 1;
    humiSen_temp_value[39] = 2;
    humiSen_humi_value[39] = 2;
    water_immer_value[19] = 3;
    gate_magn_value[19] = 4;
    tempSen_volt_value[99] = 5;
    tempHumiSen_volt_value[39] = 6; 
    waterImmer_volt_value[19] = 7;
    gateMagn_volt_value[19] = 8;
}
//----------------------------------------------------------------------------//
//函数功能：Modbus处理报文,帧错误则丢弃
//入口参数：无
//出口参数：无
//最后修改：
//备注：
//----------------------------------------------------------------------------//
int Slave_handle_recvFrame(uchar *recv_buff,ushort recv_len,uchar *send_buff)
{
    uchar Modbus_CRC_Rcv_Hi; //接收到的ModbusCRC校验码高字节
    uchar Modbus_CRC_Rcv_Lo; //接收到的ModbusCRC校验码低字节
    ushort Modbus_CRC_Rcv;   //接收到的ModbusCRC校验码
    ushort Modbus_CRC_Cal;   //根据接收到的数据计算出来的CRC值
    
    //----------------------------------------------------------//
    //开始解析
    //----------------------------------------------------------//
    if(recv_buff == NULL || send_buff == NULL || recv_len < 5) //如果接收到的一帧的字节数大于4 首先确保帧的长度在正常范围
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
                Slave_ReadHoldingReg_Process(recv_buff,send_buff); //读保持寄存器处理
                break;     

            case WRITE_SINGLE_REG://写单个保持寄存器
                Slave_WriteSingleReg_Process(recv_buff,send_buff);//写单个保持寄存器处理
                break; 

            case WRITE_MULTIPLE_REG://写多个保持寄存器
                Slave_WriteMultipleReg_Process(recv_buff,send_buff);//写多个保持寄存器处理
                break;  

            default:
                Slave_ErrorHandling(recv_buff,send_buff,0x01);//所有功能码都不符合，则返回功能码错误异常响应报文
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
        logMsg(logErr,"modbus_crc16 error!");
        return ERROR;
    } 

    return OK;
}
//----------------------------------------------------------------------------//
//函数功能：功能码0x03，读保持寄存器
//入口参数：无
//出口参数：无
//最后修改：
//备注：
//----------------------------------------------------------------------------//
int Slave_ReadHoldingReg_Process(uchar *recv_buff,uchar *send_buff)
{
    uchar  send_cnt=0;   //发送字节数量
    ushort start_addr_reg; //要读取的寄存器起始地址
    ushort num_reg; //要读取的寄存器的数量
    ushort CRC_Cal;   //CRC校验码
    ushort i, j;     //临时变量
    
    if(recv_buff == NULL || send_buff == NULL)
    {
        logMsg(logErr,"recv_buff is NULL or send_buff is NULL");
        return ERROR;
    }
    //从接收数据缓冲区得到要读取的寄存器起始地址
    start_addr_reg = MAKEWORD(recv_buff[3],recv_buff[2]);
    //从接收数据缓冲区得到要读取的寄存器数量 
    num_reg = MAKEWORD(recv_buff[5],recv_buff[4]);  
    //寄存器起始地址在温度传感器温度值的寄存器正确范围内
    if(start_addr_reg >= TEMP_START_ADDR && start_addr_reg < TEMP_END_ADDR) 
    {
        if(start_addr_reg + num_reg -1 <= TEMP_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 1) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 1; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - TEMP_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 2)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(temp_value[i]);
                send_buff[j + 1] = LOBYTE(temp_value[i]);
            }
            
            CRC_Cal = crc16(send_buff, 3 + (num_reg << 1)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 1)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 1) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
             TransData(send_buff,send_cnt);
            //*************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    } 
    //寄存器起始地址在温湿度传感器温湿度值的寄存器正确范围内
    else if(start_addr_reg >= TEMP_HUMI_START_ADDR && start_addr_reg < TEMP_HUMI_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= TEMP_HUMI_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 2) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 2; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - TEMP_HUMI_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 4)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(humiSen_temp_value[i]);//温度
                send_buff[j + 1] = LOBYTE(humiSen_temp_value[i]);
                send_buff[j + 2] = HIBYTE(humiSen_humi_value[i]);//湿度
                send_buff[j + 3] = LOBYTE(humiSen_humi_value[i]);
            }

            CRC_Cal = crc16(send_buff, 3 + (num_reg << 2)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 2)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 2) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在水浸传感器状态值的寄存器正确范围内
    else if(start_addr_reg >= WATER_IMMER_START_ADDR && start_addr_reg < WATER_IMMER_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= WATER_IMMER_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + num_reg +2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg ; //寄存器字节数量 等于 寄存器数量
           
            start_addr_reg  = start_addr_reg - WATER_IMMER_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 1)//读取寄存器的数据
            {
                send_buff[j] = water_immer_value[i];
            }

            CRC_Cal = crc16(send_buff, 3 + num_reg); //计算发送数据的CRC校验码
            send_buff[3 + num_reg ] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + num_reg + 1] = HIBYTE(CRC_Cal);//后是高字节

            //*************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在门磁传感器状态值的寄存器正确范围内
    else if(start_addr_reg >= GATE_MAGN_START_ADDR && start_addr_reg < GATE_MAGN_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= GATE_MAGN_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + num_reg  + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - GATE_MAGN_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 1)//读取寄存器的数据
            {
                send_buff[j] = gate_magn_value[i];
            }

            CRC_Cal = crc16(send_buff, 3 + num_reg); //计算发送数据的CRC校验码
            send_buff[3 + num_reg] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + num_reg + 1] = HIBYTE(CRC_Cal);//后是高字节

            //*************************************************
            TransData(send_buff,send_cnt);
            //*************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在温度传感器内部电压的寄存器正确范围内
    else if(start_addr_reg >= TEMP_VOLTA_START_ADDR && start_addr_reg < TEMP_VOLTA_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= TEMP_VOLTA_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 1) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 1; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - TEMP_VOLTA_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 2)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(tempSen_volt_value[i]);
                send_buff[j + 1] = LOBYTE(tempSen_volt_value[i]);
            }

            CRC_Cal = crc16(send_buff, 3 + (num_reg << 1)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 1)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 1) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在温湿度传感器内部电压的寄存器正确范围内
    else if(start_addr_reg >= TEMP_HUMI_VOLTA_START_ADDR && start_addr_reg < TEMP_HUMI_VOLTA_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= TEMP_HUMI_VOLTA_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 1) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 1; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - TEMP_HUMI_VOLTA_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 2)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(tempHumiSen_volt_value[i]);
                send_buff[j + 1] = LOBYTE(tempHumiSen_volt_value[i]);
            }

            CRC_Cal = crc16(send_buff, 3 + (num_reg << 1)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 1)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 1) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在水浸传感器内部电压的寄存器正确范围内
    else if(start_addr_reg >= WATER_IMMER_VOLTA_START_ADDR && start_addr_reg < WATER_IMMER_VOLTA_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= WATER_IMMER_VOLTA_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 1) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 1; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - WATER_IMMER_VOLTA_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 2)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(waterImmer_volt_value[i]);
                send_buff[j + 1] = LOBYTE(waterImmer_volt_value[i]);
            }

            CRC_Cal = crc16(send_buff, 3 + (num_reg << 1)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 1)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 1) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    //寄存器起始地址在门磁传感器内部电压的寄存器正确范围内
    else if(start_addr_reg >= GATE_MAGN_VOLTA_START_ADDR && start_addr_reg < GATE_MAGN_VOLTA_END_ADDR)
    {
        if(start_addr_reg + num_reg -1 <= GATE_MAGN_VOLTA_END_ADDR && num_reg > 0)//起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确
        {        
            send_cnt = 3 + (num_reg << 1) + 2; //计算发送字节数量 1(从站地址)+1(功能码)+1(字节数)+num_reg*2(寄存器数量)+2(crc16)

            send_buff[0] = SLAVE_ADDR; //从站地址
            send_buff[1] = READ_HOLDING_REG;//功能码
            send_buff[2] = num_reg << 1; //寄存器字节数量 等于 寄存器数量乘2
            
            start_addr_reg  = start_addr_reg - GATE_MAGN_VOLTA_START_ADDR;
            for(i = start_addr_reg, j = 3; i < start_addr_reg + num_reg; i++, j += 2)//读取寄存器的数据
            {
                send_buff[j] = HIBYTE(gateMagn_volt_value[i]);
                send_buff[j + 1] = LOBYTE(gateMagn_volt_value[i]);
            }

            CRC_Cal = crc16(send_buff, 3 + (num_reg << 1)); //计算发送数据的CRC校验码
            send_buff[3 + (num_reg << 1)] = LOBYTE(CRC_Cal);    //先是低字节
            send_buff[3 + (num_reg << 1) + 1] = HIBYTE(CRC_Cal);//后是高字节

            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************
        }
        else
        {
            logMsg(logErr,"invalid number of register!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);  //非法数据值
            return ERROR;
        }
    }
    else
    {
        logMsg(logErr,"invalid register address!");
        Slave_ErrorHandling(recv_buff,send_buff,0x02); //非法数据地址
        return ERROR;
    }

    return ERROR;
        
}

//----------------------------------------------------------------------------//
//函数功能：功能码0x06，写单个寄存器
//入口参数：无
//出口参数：无
//最后修改：2015.12.6
//备注：
//----------------------------------------------------------------------------//
int Slave_WriteSingleReg_Process(uchar *recv_buff,uchar *send_buff)
{
    uchar send_cnt;   //发送字节数量
    ushort write_addr_reg;//要写入的寄存器地址
    ushort reg_value;  //要写入的寄存器值
    ushort CRC_Cal;  //CRC校验码
    
    if(recv_buff == NULL || send_buff == NULL)
    {
        logMsg(logErr,"recv_buff is NULL or send_buff is NULL");
        return ERROR;
    }
    
    write_addr_reg = MAKEWORD(recv_buff[3],recv_buff[2]); //从接收数据缓冲区得到要写入的寄存器地址
    reg_value = MAKEWORD(recv_buff[5],recv_buff[4]); //从接收数据缓冲区得到要写入的寄存器值
    
    if(write_addr_reg >= TEMP_START_ADDR && write_addr_reg < TEMP_END_ADDR)  //寄存器起始地址在正确范围内
    {
        send_cnt = 6 + 2; //计算发送字节数量
        
        temp_value[write_addr_reg-10000] = reg_value; //将寄存器值写入对应寄存器
        
        send_buff[0] = SLAVE_ADDR; //从站地址
        send_buff[1] = WRITE_SINGLE_REG; //功能码
        send_buff[2] = HIBYTE(write_addr_reg);   //寄存器地址高字节
        send_buff[3] = LOBYTE(write_addr_reg); //寄存器地址低字节
        send_buff[4] = HIBYTE(temp_value[write_addr_reg-10000]); //寄存器值高字节
        send_buff[5] = LOBYTE(temp_value[write_addr_reg-10000]); //寄存器值低字节
        
        CRC_Cal = crc16(send_buff, 6); //计算发送数据的CRC校验码
        send_buff[6] = (uchar)(CRC_Cal >> 8);  //先是低字节
        send_buff[7] = (uchar)(CRC_Cal & 0x00FF); //后是高字节
        //**************************************************
        TransData(send_buff,send_cnt);
        //**************************************************
    } 
    else
    {
        logMsg(logErr,"invalid address of writting register!");
        Slave_ErrorHandling(recv_buff,send_buff,0x02);  //非法数据地址
        return ERROR;
    }

    return OK;
}

//----------------------------------------------------------------------------//
//函数功能：功能码0x10，写多个寄存器
//入口参数：无
//出口参数：无
//最后修改：2015.12.9
//备注：
//----------------------------------------------------------------------------//
int Slave_WriteMultipleReg_Process(uchar *recv_buff,uchar *send_buff)
{
    uchar  send_cnt; //发送字节数量
    ushort start_addr_reg;//要写入的寄存器起始地址
    ushort reg_num; //要写入的寄存器的数量
    ushort CRC_Cal;//CRC校验码
    ushort i, j;  //临时变量
    
    if(recv_buff == NULL || send_buff == NULL)
    {
        logMsg(logErr,"recv_buff is NULL or send_buff is NULL");
        return ERROR;
    }
    
    start_addr_reg = MAKEWORD(recv_buff[3],recv_buff[2]);//从接收数据缓冲区得到要写入的寄存器起始地址
    reg_num = MAKEWORD(recv_buff[5],recv_buff[4]);   //从接收数据缓冲区得到要写入的寄存器数量
    
    //温度寄存器值
    if(start_addr_reg >= TEMP_START_ADDR && start_addr_reg < TEMP_END_ADDR) //寄存器起始地址在正确范围内
    {
        if(start_addr_reg + reg_num < TEMP_END_ADDR && reg_num > 0) //起始地址+寄存器数量位于正确范围内 并且 寄存器数量正确                        
        {
            for(i = start_addr_reg, j = 7; i < start_addr_reg + reg_num; i++, j += 2) //将要写入的寄存器值写入寄存器
            {
                temp_value[i] = MAKEWORD(recv_buff[j+1] ,recv_buff[j]);
            }
            
            send_cnt = 6 + 2;
            
            send_buff[0] = SLAVE_ADDR;  //从站地址
            send_buff[1] = WRITE_MULTIPLE_REG;  //功能码
            send_buff[2] = HIBYTE(start_addr_reg); //寄存器起始地址高字节
            send_buff[3] = LOBYTE(start_addr_reg);//寄存器起始地址低字节
            send_buff[4] = HIBYTE(reg_num);  //寄存器数量高字节
            send_buff[5] = LOBYTE(reg_num);//寄存器数量低字节
            
            CRC_Cal = crc16(send_buff, 6); //计算发送数据的CRC校验码
            send_buff[6] = LOBYTE(CRC_Cal);  //先是低字节
            send_buff[7] = HIBYTE(CRC_Cal);//后是高字节
            
            //**************************************************
            TransData(send_buff,send_cnt);
            //**************************************************

        }
        else
        {
            logMsg(logErr,"invalid start register address!");
            Slave_ErrorHandling(recv_buff,send_buff,0x03);
            return ERROR;
        }
    }
    else
    {
        logMsg(logErr,"invalid multipile register address to writting !");
        Slave_ErrorHandling(recv_buff,send_buff,0x02);
        return ERROR;
    }
    return OK;
}

//----------------------------------------------------------------------------//
//函数功能：错误处理
//入口参数：ErrorType是错误类型
//出口参数：无
//最后修改：2015.12.11
//备注：
//----------------------------------------------------------------------------//
int Slave_ErrorHandling(uchar *recv_buff,uchar *send_buff,uchar ErrorType)
{
    ushort CRC_Cal; //CRC校验码

    if(recv_buff == NULL || send_buff == NULL)
    {
        logMsg(logErr,"recv_buff is NULL or send_buff is NULL");
        return ERROR;
    }    

    switch(ErrorType) //用switch分支语句来确定Modbus异常码
    {
    case 0x01:  //非法功能码
        send_buff[0] = SLAVE_ADDR;          //从站地址
        send_buff[1] = recv_buff[1] + 0x80;  //异常功能码
        send_buff[2] = 0x01;       
        //异常码 
        CRC_Cal = crc16(send_buff, 3); //计算发送数据的CRC校验码
        send_buff[3] = LOBYTE(CRC_Cal);  //先是低字节
        send_buff[4] = HIBYTE(CRC_Cal); //后是高字节

        //**************************************************
        TransData(send_buff,5);
        //**************************************************
        break;
            
    case 0x02:  //非法数据地址
        send_buff[0] = SLAVE_ADDR;  //从站地址
        send_buff[1] = recv_buff[1] + 0x80; //异常功能码
        send_buff[2] = 0x02;  //异常码
        CRC_Cal = crc16(send_buff, 3);  //计算发送数据的CRC校验码
        send_buff[3] = LOBYTE(CRC_Cal);  //先是低字节
        send_buff[4] = HIBYTE(CRC_Cal); //后是高字节
        
        //**************************************************
        TransData(send_buff,5);
        //**************************************************
        break;
            
    case 0x03: //非法数据值
        send_buff[0] = SLAVE_ADDR;  //从站地址
        send_buff[1] = recv_buff[1] + 0x80; //异常功能码
        send_buff[2] = 0x03; //异常码                             
        CRC_Cal = crc16(send_buff, 3);  //计算发送数据的CRC校验码
        send_buff[3] = LOBYTE(CRC_Cal);  //先是低字节
        send_buff[4] = HIBYTE(CRC_Cal); //后是高字节
        
        //**************************************************
        TransData(send_buff,5);
        //**************************************************
        break;
            
    default:
        logMsg(logErr,"invalid function code!");
        return ERROR;          
    }

    return OK;
}
