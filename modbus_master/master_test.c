
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <signal.h>

#include "serial_drive.h"
#include "gtypedef.h"
#include "modbus_master.h"

#ifdef LOG
    FILE *log_fd;
#endif


ushort wait_time_cnt=0;//等待时间计数
uchar enable_send = TRUE;//使能发送
static uchar state = 1; //控制流程的有限状态机变量

//信号处理函数
static void signal_handle(int signo)
{
    printf("pressed ctrl+c.\n");
    #ifdef LOG 
        if(log_fd != NULL)
        {
            fclose(log_fd);//记录报文文件描述词
        }
    #endif
    exit(0);
}
int main(int argc, char **argv) 
{
    int baudrate = 9600;
	struct termios opt;
    int flag = 0;

    //信号处理
    signal(SIGINT,signal_handle);
    //串口初始化
    flag=serial_init(&gfd,baudrate,&opt);
    if(flag==false)
    {
         printf("serial inital failed.\n");
         return -1;
     }
    #ifdef LOG
        time_t now;
        struct tm timenow;
        char date[32];
        time(&now);  //得到时间秒数
        now = now + 8*3600;
        localtime_r(&now, &timenow);  //线程安全,将秒数转化为日历，并存储在timenow结构体
        strftime(date,32,"master-%Y-%m-%d,%H:%M:%S.txt",&timenow);
        log_fd=fopen(date,"w");//创建记录报文文件
    #endif
    
    //分配内存
    m_Rxd.buf = (BYTE *)malloc(MAX_RECV_BUFF_LEN);
    m_Txd.buf = (BYTE *)malloc(MAX_SEND_BUFF_LEN);
    //内存初始化
    memset(m_Rxd.buf, 0, sizeof(BYTE) * MAX_RECV_BUFF_LEN);
    memset(m_Txd.buf, 0, sizeof(BYTE) * MAX_SEND_BUFF_LEN);
    while(1)
    {
        if(enable_send == TRUE)
        {
            wait_time_cnt = 0;
            enable_send = FALSE;
        
            switch(state) //查询状态机值,流程控制
            {
            case 1:
                logMsg(logInfo,"start request 1");
                Master_ReadHoldingReg_Request(m_Txd.buf,TEMP_START_ADDR,TEMP_DATA_LEN);
                break;
            case 2:
                logMsg(logInfo,"start request 2");
                Master_ReadHoldingReg_Request(m_Txd.buf,TEMP_HUMI_START_ADDR,TEMPHUMI_DATA_LEN);
                break;
            case 3:
                logMsg(logInfo,"start request 3");
                Master_ReadHoldingReg_Request(m_Txd.buf,WATER_IMMER_START_ADDR,WATER_IMMER_DATA_LEN);
                break;
            case 4:
                logMsg(logInfo,"start request 4");
                Master_ReadHoldingReg_Request(m_Txd.buf,GATE_MAGN_START_ADDR,GATE_MAGN_DATA_LEN);
                break;
            case 5:
                logMsg(logInfo,"start request 5");
                Master_ReadHoldingReg_Request(m_Txd.buf,TEMP_VOLTA_START_ADDR,TEMP_VOLTA_DATA_LEN);
                break;
            case 6:
                logMsg(logInfo,"start request 6");
                Master_ReadHoldingReg_Request(m_Txd.buf,TEMP_HUMI_VOLTA_START_ADDR,TEMP_HUMI_VOLTA_DATA_LEN);
                break;
            case 7:
                logMsg(logInfo,"start request 7");
                Master_ReadHoldingReg_Request(m_Txd.buf,WATER_IMMER_VOLTA_START_ADDR,WATER_IMMER_VOLTA_DATA_LEN);
                break;
            case 8:
                logMsg(logInfo,"start request 8");
                Master_ReadHoldingReg_Request(m_Txd.buf,GATE_MAGN_VOLTA_START_ADDR,GATE_MAGN_VOLTA_DATE_LEN);
                break;
            default:
                logMsg(logErr,"state error!");
            }
        }

        //查询接收
        if((m_Rxd.len = RecvData(m_Rxd.buf, MAX_RECV_BUFF_LEN)) > 0)
        {
            enable_send = TRUE;//使能发送
            wait_time_cnt = 0;//等待时间计数
            state++;//状态量+1
            Master_handle_recvFrame(m_Rxd.buf,m_Rxd.len,m_Txd.buf);  
            sleep(1);//延时1ms再召唤
        }
    
        if(state > 8)
        {
            state = 1;
            sleep(3);
        }
    }
    close_serial(gfd);
    return 0;
}