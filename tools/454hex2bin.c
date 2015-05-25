/*
    command-line tool to convert PIC16F1454 Intel Hex to DFU binary

    Copyright (C) 2015 Peter Lawrence

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
#include <string.h>
#include <malloc.h>

#define PM_SIZE 16384

static unsigned readhex(const char *text, unsigned digits);

int main(int argc, char *argv[])
{
	FILE *input, *output;
	char line[256];
	unsigned count, address;
	const char *ptr;
	struct
	{
		unsigned out_of_bounds:1;
		unsigned extended:1;
	} flags;
	unsigned char *image;

	if (argc < 3)
	{
		fprintf(stderr, "%s <input_srec> <output_binary>\n", argv[0]);
		return -1;
	}

	image = (unsigned char *)malloc(PM_SIZE);

	if (NULL == image)
	{
		fprintf(stderr, "ERROR: unable to allocate memory\n");
		return -1;
	}

	input = fopen(argv[1], "rb");

	if (NULL == input)
	{
		fprintf(stderr, "ERROR: unable to open input file %s\n", argv[1]);
		return -1;
	}

	output = fopen(argv[2], "wb");

	if (NULL == output)
	{
		fprintf(stderr, "ERROR: unable to open output file %s\n", argv[2]);
		return -1;
	}

	for (address = 0; address < PM_SIZE; address += 2)
	{
		image[address + 0] = 0xFF;
		image[address + 1] = 0x3F;
	}

	memset(&flags, 0, sizeof(flags));

	while (!feof(input))
	{
		if (fgets(line, sizeof(line), input))
		{
			if (':' == line[0])
			{
				count = readhex(line + 1, 2);
				address = readhex(line + 3, 4);
				if (0 == strncmp(line + 7, "00", 2) && !flags.extended) /* data record */
				{
					ptr = line + 9;
					while (count--)
					{
						if ( (address < 0x400) || (address >= 0x8000) )
							flags.out_of_bounds = 1;
						else
							image[address] = readhex(ptr, 2);
						address++; ptr += 2;
					}
				}
				else if (0 == strncmp(line + 7, "04", 2)) /* subsequent data is not program code */
				{
					flags.extended = 1;
				}
				else if (0 == strncmp(line + 7, "01", 2)) /* end of file record */
				{
					break;
				}
			}
		}
	}

	if (flags.out_of_bounds)
		fprintf(stderr, "WARNING: supplied input file is faulty and used out-of-bounds addresses\n");

	fclose(input);

	fwrite(image, 1, PM_SIZE, output);
	fclose(output);

	free(image);

	return 0;
}

static unsigned readhex(const char *text, unsigned digits)
{
	unsigned result = 0;

	while (digits--)
	{
		result <<= 4;
		
		if ( (*text >= '0') && (*text <= '9') )
			result += *text - '0';
		else if ( (*text >= 'A') && (*text <= 'F') )
			result += 10 + *text - 'A';
		else if ( (*text >= 'a') && (*text <= 'f') )
			result += 10 + *text - 'a';

		text++;
	}

	return result;
}
