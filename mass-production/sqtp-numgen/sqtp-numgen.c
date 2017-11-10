/*
    quick and dirty Microchip PIC SQTP hex digit USB serial number generator

    writes a Microchip ".num" file to stdout given hard-coded parameters

    Copyright (C) 2015,2017 Peter Lawrence

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in 
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <time.h>

#define upper_byte(x) ((x & 0xFF00) >> 8)
#define lower_byte(x) ((x & 0x00FF) >> 0)

/*
these parameters are tailored for the open-source 'USB 512-Word DFU Bootloader for PIC16(L)F1454/5/9'
said bootloader allocates a region within its memory space for an eight-character serial number
*/
const unsigned word_address      = 0x1F0;
const unsigned serial_num_count  = 4096;
const unsigned serial_num_digits = 8;
const unsigned retlw_opcode      = 0x3400;

void main(void)
{
	unsigned serial_num, index, count;
	unsigned char checksum, digit;
	unsigned instruction[2];
	unsigned num_of_bytes, byte_address;

	srand(time(NULL));
	serial_num = rand();

	for (count = 0; count < serial_num_count; count++, serial_num++)
	{
		/*
		each hex digit is represented with one unicode character
		each unicode character consumes two bytes
		each byte consumes one 14-bit PIC instruction (effectively two bytes in Intel Hex)
		*/

		num_of_bytes = 4 * serial_num_digits;
		byte_address = word_address << 1;

		checksum  = num_of_bytes;
		checksum += upper_byte(byte_address);
		checksum += lower_byte(byte_address);

		printf(":%02X%04X00", num_of_bytes, byte_address);
		
		for (index = 0; index < serial_num_digits; index++)
		{
			if (0 == index)
			{
				digit = 'R';
			}
			else
			{
				digit = (serial_num >> (4 * (serial_num_digits - index - 1))) & 0xF;
				digit = (digit < 10) ? ('0' + digit) : ('A' + (digit - 10));
			}

			/* instruction for first byte of unicode character */
			instruction[0] = retlw_opcode | digit;
			checksum += lower_byte(instruction[0]);
			checksum += upper_byte(instruction[0]);

			/* instruction for second byte of unicode character */
			instruction[1] = retlw_opcode | 0;
			checksum += lower_byte(instruction[1]);
			checksum += upper_byte(instruction[1]);

			printf("%02X%02X%02X%02X", lower_byte(instruction[0]), upper_byte(instruction[0]), lower_byte(instruction[1]), upper_byte(instruction[1]));
		}

		/* Intel Hex checksum is 2's complement of the entire record line */
		checksum = ~checksum + 1;
		printf("%02X\n", checksum);
	}

	/* indicate end of file */
	printf(":00000001FF\n");
}
