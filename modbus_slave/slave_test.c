/*
 * @Author: your name
 * @Date: 2019-12-19 23:10:31
 * @LastEditTime: 2019-12-24 19:36:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /hebi-modbus/slave_test.c
 */

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
#include "modbus_slave.h"
#ifdef LOG
    FILE *log_fd;
#endif

//信号处理函数
static void signal_handle(int signo)
{
    printf("pressed ctrl+c.\n");
    #ifdef LOG 
        if(log_fd != NULL){
            fclose(log_fd);//记录报文文件描述词
        }
    #endif
    exit(0);
}
int main(int argc, char **argv) {

    int baudrate = 9600;
	struct termios opt;
    int flag = 0;
    //信号处理
    signal(SIGINT,signal_handle);
    //串口初始化
    flag=serial_init(&gfd,baudrate,&opt);
    if(flag==false){
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
        strftime(date,32,"slave-%Y-%m-%d,%H:%M:%S.txt",&timenow);
        log_fd=fopen(date,"w");//创建记录报文文件
    #endif

    Data_init();//数据初始化
    
    //分配内存
    m_Rxd.buf = (BYTE *)malloc(MAX_RECV_BUFF_LEN);
    m_Txd.buf = (BYTE *)malloc(MAX_SEND_BUFF_LEN);
    //内存初始化
    memset(m_Rxd.buf, 0, sizeof(BYTE) * MAX_RECV_BUFF_LEN);
    memset(m_Txd.buf, 0, sizeof(BYTE) * MAX_SEND_BUFF_LEN);
    
    while(1)
    {
        if((m_Rxd.len = RecvData(m_Rxd.buf, MAX_RECV_BUFF_LEN)) > 0)
        {
            Slave_handle_recvFrame(m_Rxd.buf,m_Rxd.len,m_Txd.buf);  
        }
    }
    close_serial(gfd);
    return 0;
}