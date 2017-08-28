#include "windows.h"


HANDLE sp_open(LPSTR ComPort, DWORD BitRate, DWORD ReadTimeOut)
{
	HANDLE hComm;
	DCB dcb = {0};
	COMMTIMEOUTS CommTmOut = {0};
	
	// Get a handle to the port.
	hComm = CreateFile(ComPort,  
		GENERIC_READ | GENERIC_WRITE, 
		0, 
		NULL, 
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hComm == INVALID_HANDLE_VALUE)
		return INVALID_HANDLE_VALUE;

/*
	// Get the current state of the port
	FillMemory(&dcb, sizeof(dcb), 0);


	
	if (!GetCommState(hComm, &dcb))
	{
		return INVALID_HANDLE_VALUE;
	}
	else
	{
		dcb.BaudRate = BitRate;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.fDtrControl = DTR_CONTROL_DISABLE;
		dcb.fRtsControl = RTS_CONTROL_DISABLE;
		dcb.fOutX = FALSE;
		dcb.fInX = FALSE;
		dcb.fOutxCtsFlow = FALSE;
		dcb.fOutxDsrFlow = FALSE;
		dcb.ByteSize = 8;
	}
		
	if (!SetCommState(hComm, &dcb))
		return INVALID_HANDLE_VALUE;

*/
	// Set the timeout conditions
	if (!GetCommTimeouts(hComm, &CommTmOut))
	{
		return INVALID_HANDLE_VALUE;
	}
	else
	{
		CommTmOut.ReadIntervalTimeout = 110;
		CommTmOut.ReadTotalTimeoutMultiplier = 0;
		CommTmOut.ReadTotalTimeoutConstant = 100;
		CommTmOut.WriteTotalTimeoutMultiplier = 0;
		CommTmOut.WriteTotalTimeoutConstant = 100;
	}

	if (!SetCommTimeouts(hComm, &CommTmOut))
		return INVALID_HANDLE_VALUE;

   PurgeComm(hComm,PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);

	return hComm;
}

INT sp_close(HANDLE hComPort)
{
	return CloseHandle(hComPort);
}

int sp_write(HANDLE hComPort, unsigned char *Data, unsigned short Len)
{
	DWORD BytesWritten;

	if (!WriteFile(hComPort, Data, Len, &BytesWritten, NULL)) return -1;
	return BytesWritten;
}

int sp_read(HANDLE hComPort, unsigned char *Data, unsigned short Len)
{
	DWORD BytesRead;

	if (!ReadFile(hComPort, Data, Len, &BytesRead, NULL)) return -1;
	return BytesRead;
}

