/*
 * @Author: your name
 * @Date: 2019-12-05 19:34:15
 * @LastEditTime: 2019-12-24 19:05:36
 * @LastEditors: Please set LastEditors
 * @Description: 串口设置
 * @FilePath: /balance-101-4g/terminal_io.c
 */
#include "serial_drive.h"

int gfd; //全局串口文件描述符
#ifdef LOG
extern FILE *log_fd;
#endif
//open serial
int open_serial(char *dev)
{
    int fd;
    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY, 0);
    if (fd < 1)
    {
        logMsg(logErr, "open <%s> error %s\n", dev, strerror(errno));
        return -1;
    }

    return fd;
}

//close serial
void close_serial(int pf)
{
    close(pf);
}
/**
 * @description: 设置串口参数
 * @param {fd：设备文件，databits：数据位，stopbits：停止位，parity：校验位} 
 * @return: 返回true，串口设置成功。
 */
int set_termios(int fd, struct termios *options, int databits, int stopbits, int parity)
{

    if (tcgetattr(fd, options) != 0)
    {
        logMsg(logErr, "SetupSerial 1");
        return (false);
    }
    options->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options->c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON); //串口接收到0x11/0x13不忽略
    options->c_oflag &= ~OPOST;
    options->c_cflag &= ~CSIZE;
    /* No hardware flow control */
    options->c_cflag &= ~CRTSCTS;
    switch (databits)
    { /*设置数据位数*/
    case 7:
        options->c_cflag |= CS7;
        break;
    case 8:
        options->c_cflag |= CS8;
        break;
    default:
        logMsg(logErr, "Unsupported data size\n");
        return (false);
    }
    switch (parity)
    {
    case 'n':
    case 'N':
        options->c_cflag &= ~PARENB; /* Clear parity enable */
        options->c_iflag &= ~INPCK;  /* disnable parity checking */
        break;
    case 'o':
    case 'O':
        options->c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        options->c_iflag |= INPCK;             /* enable parity checking */
        break;
    case 'e':
    case 'E':
        options->c_cflag |= PARENB;  /* Enable parity */
        options->c_cflag &= ~PARODD; /* 转换为偶效验*/
        options->c_iflag |= INPCK;   /* enable parity checking */
        break;
    case 'S':
    case 's': /*as no parity*/
        options->c_cflag &= ~PARENB;
        options->c_cflag &= ~CSTOPB;
        break;
    default:
        logMsg(logErr, "Unsupported parity\n");
        return (false);
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options->c_cflag &= ~CSTOPB;
        break;
    case 2:
        options->c_cflag |= CSTOPB;
        break;
    default:
        logMsg(logErr, "Unsupported stop bits\n");
        return (false);
    }

    /* Set input parity option */
    if (parity != 'n')
        options->c_iflag |= INPCK;
    /* Output mode */
    options->c_oflag = 0;

    /* No active terminal mode */
    options->c_lflag = 0;

    /* Overflow data can be received, but not read */
    if (tcflush(fd, TCIFLUSH) < 0)
    {
        logMsg(logErr, "tcflush failed");
        return (false);
    }

    if (tcsetattr(fd, TCSANOW, options) < 0)
    {
        logMsg(logErr, "SetupSerial failed");
        return (false);
    }
    return (true);
}

//设置串口波特率
int set_baudrate(int fd, struct termios *opt, int baudrate)
{
    /* Input baud rate */
    if (cfsetispeed(opt, baudrate) < 0)
        return false;
    /* Output baud rate */
    if (cfsetospeed(opt, baudrate) < 0)
        return false;
    /* Overflow data can be received, but not read */
    if (tcflush(fd, TCIFLUSH) < 0)
        return false;
    if (tcsetattr(fd, TCSANOW, opt) < 0)
        return false;

    return true;
}
//查找对应的波特率
int find_baudrate(int rate)
{
    int baudr;

    switch (rate)
    {
    case 600:
        baudr = B600;
        break;
    case 1200:
        baudr = B1200;
        break;
    case 9600:
        baudr = B9600;
        break;
    case 19200:
        baudr = B19200;
        break;
    case 38400:
        baudr = B38400;
        break;
    case 57600:
        baudr = B57600;
        break;
    case 115200:
        baudr = B115200;
        break;
    case 230400:
        baudr = B230400;
        break;
    case 460800:
        baudr = B460800;
        break;
    case 500000:
        baudr = B500000;
        break;
    case 576000:
        baudr = B576000;
        break;
    default:
        logMsg(logInfo, "invalid baudrate, set baudrate for 115200\n");
        baudr = B115200;
        break;
    }

    return baudr;
}
/* send len bytes data in buffer */
int serial_write(int fd, const void *buf, int len, struct timeval *write_tv)
{
    int count;
    int ret;
    fd_set output;

    ret = 0;
    count = 0;
    FD_ZERO(&output);
    FD_SET(fd, &output);
    do
    { /* listen */
        ret = select(fd + 1, NULL, &output, NULL, write_tv);
        if (ret == -1)
        { /* error */
            logMsg(logErr, "select() failed!");
            return ERROR;
            ;
        }
        else if (ret)
        { /* write buffer */
            ret = write(fd, (BYTE *)buf + count, len);
            if (ret < 1)
            {
                logMsg(logErr, "write error %s\n", strerror(errno));
                return ERROR;
                ;
            }
            count += ret;
            len -= ret;
        }
        else
        { /* timeout */
            logMsg(logErr, "time out.\n");
            return ERROR;
        }
    } while (len > 0);
    return count;
}
/**
 * @description:发送报文 
 * @param {type} 
 * @return: 返回发送数据的长度，返回-1错误
 */
int TransData(BYTE *buff, int bufLen)
{
    int write_size;
    struct timeval tv;

    tv.tv_sec = 2;
    tv.tv_usec = 0;
    write_size = serial_write(gfd, buff, bufLen, &tv);

    if (write_size <= 0 || write_size != bufLen)
    {
        logMsg(logErr, "Transmited data error,data size of transmission=%d\n", write_size);
        return 0;
    }
#ifdef LOG
    if (log_fd != NULL)
    {
        struct timeval time1; //精确到微秒
        struct tm timenow;    //实例化tm结构指针
        char date[32];

        gettimeofday(&time1, NULL); //获取微秒
        time1.tv_sec = time1.tv_sec + 8 * 3600;
        localtime_r(&time1.tv_sec, &timenow); //线程安全,将秒数转化为日历，并存储在timenow结构体

        strftime(date, 32, "%H:%M:%S", &timenow);                //将时间转化为自己需要的时间格式
        sprintf(date + 8, ".%03d", (int)(time1.tv_usec / 1000)); //将微秒追加到时间后面
        fprintf(log_fd, "%s send:%d=> ", date, write_size);
    }
#endif
    printf("send:%d=> ", write_size);
    for (int i = 0; i < write_size; i++)
    {
        printf("%02x ", (int)buff[i]);
#ifdef LOG
        if (log_fd != NULL)
        {
            fprintf(log_fd, "%02x ", (int)buff[i]);
        }

#endif
    }
#ifdef LOG
    if (log_fd != NULL)
    {
        fprintf(log_fd, "\n");
    }
#endif
    printf("\n");
    //logMsg(logInfo,"Transmited data size:%d",write_size);
    return write_size;
}
/**
 * @description:读串口不定长数据读,假设两个数据之间最大时间间隔1ms,等待1ms后下一数据还没到来说明一帧数据读完
 * @param {type} 
 * @return: 
//  */
extern ushort wait_time_cnt;//等待时间计数
extern uchar enable_send;//使能发送
int serial_read(int fd, const void *data, int bufLen, int timeout_sec)
{
    int cnt = 0;
    int read_size = 0;
    struct timeval timeout;
    fd_set rfds;

    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    switch (select(fd + 1, &rfds, NULL, NULL, &timeout))
    {
    case -1:
        logMsg(logErr, "select()\n");
        return ERROR;
    case 0:
        wait_time_cnt++;
        if(wait_time_cnt >= 5) //超时3s未收到数据,重发请求
        {
            wait_time_cnt = 0;
            enable_send = TRUE;//使能发送
        }
        return ERROR;
    default:
        if (FD_ISSET(fd, &rfds)) //判断fd是否可读。
        {
            do
            {
                read_size = read(fd, (BYTE *)data + cnt, bufLen);
                if (read_size > 0)
                {
                    cnt += read_size;
                    bufLen -= read_size;
                }
                usleep(100000); //延时等待下一个数据到来
            } while (read_size > 0);
        }
    }
    return cnt;
}
/**
 * @description: 接收报文
 * @param {type} 
 * @return: 返回-1，接收错误，，，否则返回接收到的数据长度
 */
int RecvData(BYTE *buff, int bufLen)
{
    int read_size;

    read_size = serial_read(gfd, (BYTE *)buff, bufLen, 1);//等待2s
    if (read_size > 0)
    { //接收到数据
        printf("recv:%d<= ", read_size);
        #ifdef LOG
            if (log_fd != NULL)
            {
                struct timeval time1; //精确到微秒
                struct tm timenow;    //实例化tm结构指针
                char date[32];

                gettimeofday(&time1, NULL); //获取微秒
                time1.tv_sec = time1.tv_sec + 8 * 3600;
                localtime_r(&time1.tv_sec, &timenow);                    //线程安全,将秒数转化为日历，并存储在timenow结构体
                strftime(date, 32, "%H:%M:%S", &timenow);                //将时间转化为自己需要的时间格式
                sprintf(date + 8, ".%03d", (int)(time1.tv_usec / 1000)); //将微秒追加到时间后面
                fprintf(log_fd, "%s recv:%d<= ", date, read_size);
         }
        #endif
        for (int i = 0; i < read_size; i++)
        {
            printf("%02x ", (int)buff[i]);
            #ifdef LOG
                if (log_fd != NULL){
                    fprintf(log_fd, "%02x ", (int)buff[i]);
                }
                #endif
        }
        #ifdef LOG
            if (log_fd != NULL){
                fprintf(log_fd, "\n");
            }
        #endif
        printf("\n");
        return read_size;
    }
    return 0;
}
/**
 * @description: 串口初始化
 * @param {type} 
 * @return: 
 */
int serial_init(int *fd, int baud, struct termios *opt)
{
    int state;
    int baudrate;

    /* open serial */
    *fd = open_serial("/dev/ttyPS1");
    if (*fd < 1){
        return false;
    }
    //串口设置，8位数据位，1位停止位，无校验
    state = set_termios(*fd, opt, 8, 1, 'e');
    if (state == false){
        logMsg(logErr, "set termios failed\n");
        return false;
    }
    //串口终端波特率设置
    baudrate = find_baudrate(baud);
    state = set_baudrate(*fd, opt, baudrate);
    if (state == false){
        logMsg(logErr, "set usart baudrate of 115200 faild");
        return false;
    }
    return true;
}