//
// usb16f1prog
// Copyright (c) 2017, David Pribyl
// v1.0, 28.8.2017
//
// This file is distributed  under the terms of the GNU 
// General Public License (GPL).   See the accompanying file "COPYING" 
// for more details.
//
// Uploads firmware to a PIC16F1xxx microcontroller programmed with
// Matt Sarnoff's USB bootloader.
//
// Accepts 16-bit Intel HEX files as input.
// Can only be used to write to program memory; the bootloader does not support
// writing the configuration words or user ID words, so values at those
// addresses in the input file are ignored.
//
// The programming protocol is very simplistic. There are three requests,
// distinguished by their length: Set Parameters (4 bytes), Write (64 bytes),
// and Reset (1 byte). After a Set Parameters or Write command, the host (e.g.
// this script) must wait for a 1-byte status response. If the response byte is
// 0x01, the operation succeeded. Otherwise, the host should abort. (See below
// for possible error values.)
//
// Set Parameters is 4 bytes long:
// - addressLowByte
// - addressHighByte
// - expectedChecksum
// - shouldErase
// addressLowByte/HighByte is the 16-bit word address, aligned to a 32-word
// boundary, where data should be written.
// expectedChecksum is the 8-bit checksum of the 32 words to be written at the
// specified address. (This is the 2's complement of the byte-wise sum mod 256
// of the upcoming 32 words.)
// If the shouldErase byte is 0x45 ('E'), the flash row at that address is
// erased. An erase is mandatory before a Write command.
//
// Write is 64 bytes long:
// - dataWord0LowByte
// - dataWord0HighByte
// ...
// - dataWord31LowByte
// - dataWord31HighByte
// In other words, exactly 32 words, little-endian. If less than 32 words are
// to be written, the sequence should be padded out with 0x3FFF (0xFF, 0x3F).
// The device may return an error if the checksum of the data does not match
// the value sent in the last Set Parameters command, or if the values in flash
// do not match the supplied data after the write. (The latter may happen if you
// attempt to write to an address outside the device's ROM space, or to the 
// bootloader region.)
//
// Reset is 1 byte long. If it is 0x52 ('R'), the device is reset, and no status
// is returned.

#include <windows.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "sp.h"


//#define TEST 1

#define HE_EOF  -1      /* unexpected EOF */
#define HE_DEX  -2      /* hex digit required */
#define HE_CEX  -3      /* missing ':' */
#define HE_CHK  -4      /* checksum error */
//#define HE_IGN  1       /* warning that some records were ignored */

#define STATUS_OK  1
#define STATUS_INVALID_COMMAND  2
#define STATUS_INVALID_CHECKSUM  3
#define STATUS_VERIFY_FAILED  4

#define BCMD_ERASE  0x45
#define BCMD_RESET  0x52


#define MAXPROG	8192*2


unsigned char progmem[MAXPROG];

static int check;
static int fail;

/*****************************************************************************************/
static unsigned hexdigit(FILE *fp)
{
	int c;
	
	if ( fail )
		return 0;
	
	if ( (c=getc(fp)) == EOF ) {
		fail = HE_EOF;			/* unexpected EOF */
		return 0;
	}
	
	c -= c>'9'? 'A'-10: '0';
	if ( c<0 || c>0xF )
		fail = HE_DEX;			/* hex digit expected */
	return c;
}

/*****************************************************************************************/

static unsigned hexbyte(FILE *fp)
{
	unsigned b;
	
	b = hexdigit(fp);
	b = (b<<4) + hexdigit(fp);
	check += b;
	return b;
}

/*****************************************************************************************/

static unsigned hexword(FILE *fp)
{
	unsigned w;
	
	w = hexbyte(fp);
	w = (w<<8) + hexbyte(fp);
	return w;
}

/*****************************************************************************************/

#define HEXBYTE()	hexbyte(fp); if (fail) return fail
#define HEXWORD()	hexword(fp); if (fail) return fail


/*****************************************************************************************/

int loadhex(FILE *fp)
{
    int address;
	int adrhi=0;
	int type;
	int linelen;
	int i;
   unsigned char b;
	
   fail = 0;

	memset(progmem,0xff,MAXPROG);
	
    type = 0;
    while ( type != 1 ) {
		if ( getc(fp) != ':' ) return HE_CEX;		/* expected ':' */
		check = 0;
		linelen = HEXBYTE();
		address = HEXWORD();
		type	= HEXBYTE();
		
		if (type==0) { //Data 
			if (adrhi==0) { //progmem
				for ( i=0; i<linelen; i++) {
					b=HEXBYTE();
					if (address<MAXPROG) progmem[address++]=b;
				}
			}
		}
		
		else if (type==4) { //Addr
			adrhi = HEXWORD();
		}
		HEXBYTE();			/* get checksum */
		(void) getc(fp);			/* discard end-of-line */
		if ( check&0xFF )
			return HE_CHK; 		/* checksum error */
		
	}
	
   return 0;
}

/*****************************************************************************************/
int testblock(unsigned char * mem) {
	int j;
	for (j=0;j<64;j++) {
		if (mem[j]!=0xFF) return 1;
	}
	return 0;
}
/*****************************************************************************************/
unsigned char CompBlockChKSum(unsigned char * mem) {

	int j=0;
	unsigned char chksum=0;


	while (j<64) {
		chksum+=mem[j++];
		chksum+=mem[j++]&0x3F;
	}
	chksum=-chksum;
	return chksum;

}

/*****************************************************************************************/
void device_error(unsigned char Status) {
/*	
	switch (Status) {
	case STATUS_INVALID_COMMAND:     printf("invalid command\n");
		break;
	case STATUS_INVALID_CHECKSUM:    printf("checksum failed; data not written\n");
		break;
	case STATUS_VERIFY_FAILED:       printf("write verification failed\n");
		break;
	}
*/	
}

int device_set_params(HANDLE hComPort, int  addr, unsigned char checksum) {
	
	unsigned char Buff[4];
	unsigned char Status;
	int Back=1; //Error
	int ret;

	Buff[0]=addr&0xFF;
	Buff[1]=(addr>>8)&0xFF;
	Buff[2]=checksum;
	Buff[3]=BCMD_ERASE;

#if TEST
	Sleep(10);
	Back=0;
#else
	ret=sp_write(hComPort,Buff,sizeof(Buff));
	
	if (ret==sizeof(Buff)) {
		ret=sp_read(hComPort,&Status,1);
		if (ret==1) {
			if (Status==STATUS_OK) Back=0;
			else device_error(Status);
			
		}
	}
#endif		
	return Back;	
}


int device_write(HANDLE hComPort, unsigned char * Row) {
	unsigned char Buff[64];
	unsigned char Status;
	int Back=1; //Error
	int i,ret;

	
	for(i=0;i<64;i+=2) {
		Buff[i]=Row[i];
		Buff[i+1]=Row[i+1]&0x3F;
		
	}
#if TEST
   Sleep(50);
	Back=0;
#else
	ret=sp_write(hComPort,Buff,sizeof(Buff));
	
	if (ret==sizeof(Buff)) { //64
		ret=sp_read(hComPort,&Status,1);
		if (ret==1) {
			if (Status==STATUS_OK) Back=0;
			else device_error(Status);
			
		}
	}
#endif		
	return Back;	
}

int device_reset(HANDLE hComPort) {
	unsigned char Cmd;
	int Back=1; //Error
	int ret;

	Cmd=BCMD_RESET;
	ret=sp_write(hComPort,&Cmd,1);

	if (ret==1) Back=0;

	return Back;	
}



/*****************************************************************************************/
unsigned char ProgLine[64+3];

void PrepareProgressLine(void) {
	ProgLine[0]='[';
	memset(ProgLine+1,'B',4);
	memset(ProgLine+1+4,'-',64-4);
	ProgLine[65]=']';
	ProgLine[66]=0;

}
void ShowProgressLine(int Addr,char Action) {
	int Pos=Addr/(32*4);
	ProgLine[Pos+1]=Action;
	printf("\r  %s",ProgLine);
}

int main(int argc, char *argv[]) {
	
	int ret=-1;
	FILE *fp;
	int startaddr=0;
	int endaddr=0;
	int i;
	HANDLE ComPort;
	
	if (argc > 2) {
		printf ("usb16f1prog:\n");

                printf ("  Loading hex file ...\n");
		fp=fopen(argv[2],"r");
		if (fp!=NULL) {
			ret=loadhex(fp);
			fclose(fp);
		}	
	
		if (ret!=0) {
                	printf("\nError - unable to read '%s' file\n",argv[2]);

		} else	{ //Analize progmem
			for (i=0;i<MAXPROG;i+=64) {
				int usedblock;
				
				usedblock=testblock(progmem+i);
				
				if ((startaddr==0)&&(usedblock)) startaddr=i;
				if (usedblock) endaddr=i;
				
			}
			printf("  Start: 0x%04x / End: 0x%04x\n",startaddr/2,endaddr/2);
			PrepareProgressLine();

			if (startaddr/2<0x200) {
				printf("\nErrror - code start om bootloader memory\n");

			} else {

	/*
				for (i=startaddr;i<=endaddr;i+=64) {
					CompBlockChKSum(progmem+i);
				}
	*/
				
				ComPort = sp_open(argv[1], 9600, 1000);
				if (ComPort==INVALID_HANDLE_VALUE) {

				   printf("\nError - unable to open '%s' serial port\n",argv[1]);
				}else {
					int ret=0;

					printf("\n  Programmig...\n");
	
					for (i=startaddr;i<=endaddr;i+=64) {
						unsigned char checksum;

						checksum=CompBlockChKSum(progmem+i);

						ShowProgressLine(i/2,'E');
						ret=device_set_params(ComPort, i/2, checksum);
						//printf("device_set_params(%04X):%d\n",i/2,ret);
						if (ret) break;
						ShowProgressLine(i/2,'W');
						ret=device_write(ComPort, progmem+i);
						//printf("device_set_params():%d\n",ret);

						if (ret) break;

				}
				if (!ret) {
					for (i=endaddr+64;i<MAXPROG;i+=64) {
						ShowProgressLine(i/2,'E');
						ret=device_set_params(ComPort, i/2, 0); //just errase
						//printf("device_set_params(%04X):%d\n",i/2,ret);
						if (ret) break;
					}
				}
				if (!ret) {
					device_reset(ComPort);
				}

				sp_close(ComPort);

				if (!ret) {
					printf("\n\nDone.\n");
				} else {
					printf("\n\nError on address: 0x%04x.\n",i/2);
				}

				

				}
			}
		}
		
	} else {	 //argc >2
        	printf ("\nUsage:\n");
        	printf ("   usb16f1prog com1 file.hex\n");
               	printf ("     com1     .. bootloader serial port\n");
               	printf ("     file.hex .. program to write\n");
		printf ("\n");
	}
	return 0;
}



