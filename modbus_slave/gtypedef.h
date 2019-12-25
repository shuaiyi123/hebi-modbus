/*
 * @Author: Yangxl
 * @LastEditors: Please set LastEditors
 * @Description: 数据类型定义文件
 * @Date: 2019-03-28 21:06:03
 * @LastEditTime: 2019-09-16 01:42:56
 */

#ifndef gtypedef__h
#define gtypedef__h

#ifdef __cplusplus
extern "C"
{
#endif

#if !defined(ON) || (ON != 1)
#define ON 1
#endif

#if !defined(OFF) || (OFF != 0)
#define OFF 0
#endif

#if !defined(OK) || (OK != 0)
#define OK 0
#endif

#if !defined(ERROR) || (ERROR != -1)
#define ERROR (-1)
#endif

#if !defined(TRUE) || (TRUE != 1)
#define TRUE 1
#endif

#if !defined(FALSE) || (FALSE != 0)
#define FALSE 0
#endif

#if !defined(true) || (true != 1)
#define true 1
#endif

#if !defined(false) || (false != 0)
#define false 0
#endif

#ifndef BOOL
#define BOOL bool
#endif

#if 1
#ifndef BYTE
    typedef unsigned char BYTE;
#endif

#ifndef WORD
    typedef unsigned short WORD;
#endif

#ifndef DWORD
    typedef unsigned long DWORD;
#endif
#endif

#ifndef uchar
    typedef unsigned char uchar;
#endif

#ifndef uint
    typedef unsigned int uint;
#endif

#ifndef ushort
    typedef unsigned short ushort;
#endif

#ifndef ulong
    typedef unsigned long ulong;
#endif

//define storage type operate that's independence CPU type
#define LOBYTE(w) ((BYTE)(w))
#define HIBYTE(w) ((BYTE)((WORD)(w) >> 8))
#define LOWORD(l) ((WORD)(l))
#define HIWORD(l) ((WORD)((DWORD)(l) >> 16))
#define MAKELONG(low, high) ((DWORD)(((WORD)(low)) | (((DWORD)((WORD)(high))) << 16)))
#define MAKEDWORD(ll, lh, hl, hh) (MAKELONG(MAKEWORD(ll, lh), MAKEWORD(hl, hh)))
#define MAKEWORD(low, high) ((WORD)(((BYTE)(low)) | (((WORD)((BYTE)(high))) << 8)))
#define MAKEADDR(ptr) ((LONG)((LPBYTE)ptr))
#define LLBYTE(w) ((BYTE)(w))
#define LHBYTE(w) ((BYTE)((WORD)(w) >> 8))
#define HLBYTE(w) ((BYTE)((DWORD)(w) >> 16))
#define HHBYTE(w) ((BYTE)((DWORD)(w) >> 24))

//define basic type exchange operate
#define PTRTOINT(x) (*((UINT far *)x))
#define PTRTODWORD(x) (*((DWORD far *)x))
#define TOBYTE(x) (*((LPBYTE)&x))
#define TOUINT(x) (*((LPUINT)&x))
#define TODWORD(x) (*((LPDWORD)&x))
#define TOWORD(x) (*((LPWORD)&x))
#define ArraySize(a) (sizeof(a) / sizeof(a[0]))

#define BCD2INT(n) ((n) - ((n) / 16) * 6)
#define INT2BCD(n) ((n) + ((n) / 10) * 6)

#define PRESENT_BIT32(x) (((uint32)((uint32)1 << (x)))) //指定DWORD中某位
#define BIT_ON32(m, b) (((m)&PRESENT_BIT32(b)) != 0)    //检测DWORD中指定位是否为1
#define SET_BIT32(m, b) ((m) |= PRESENT_BIT32(b))       //置DWORD中指定位
#define CLR_BIT32(m, b) ((m) &= ~PRESENT_BIT32(b))      //清DWORD中指定位

#ifdef __cplusplus
}
#endif

#endif