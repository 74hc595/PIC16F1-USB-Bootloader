#ifndef __SP_H
#define __SP_H

#ifdef __cplusplus
extern "C"
{
#endif

HANDLE sp_open(LPSTR ComPort, DWORD BitRate, DWORD ReadTimeOut);
INT sp_close(HANDLE hComPort);
int sp_write(HANDLE hComPort, unsigned char *Data, unsigned short Len);
int sp_read(HANDLE hComPort, unsigned char *Data, unsigned short Len);

#ifdef __cplusplus
}
#endif

#endif

