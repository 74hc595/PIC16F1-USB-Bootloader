/*
    command-line tool to merge several PIC16F1454 Intel Hex files
    Copyright (C) 2015 Peter Lawrence

    say you want to send a single Intel Hex file to production that 
	combines both a bootloader and your upgradeable application

    this utility performs such a merging operation

    PIC config bits are cherry-picked from the first file (bootloader) only

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
#include <assert.h>

struct memory_blob
{
	unsigned address, count;
	unsigned char *data;
	struct memory_blob *next;
};

static unsigned readhex(const char *text, unsigned digits);
static struct memory_blob *find_blob(unsigned address, unsigned count, struct memory_blob **list);
static unsigned checksum_passes(const char *text);
static void print_blobs(FILE *handle, struct memory_blob *blob);

int main(int argc, char *argv[])
{
	FILE *handle;
	char line[256];
	unsigned address, upper_address, offset;
	unsigned count, lineno;
	const char *ptr;
	int index;
	struct memory_blob *blob, *pm_list, *config_list;

	pm_list = NULL;
	config_list = NULL;

	if (argc < 4)
	{
		fprintf(stderr, "%s <bootloader_ihex> <input2_ihex> [... <inputN_ihex>] <output_ihex>\n", argv[0]);
		return -1;
	}

	for (index = 1; index < (argc - 1); index++)
	{
		handle = fopen(argv[index], "rb");

		if (NULL == handle)
		{
			fprintf(stderr, "ERROR: unable to open input file %s\n", argv[index]);
			return -1;
		}

		upper_address = 0; lineno = 0;

		while (!feof(handle))
		{
			lineno++;

			if (fgets(line, sizeof(line), handle) <= 0)
				continue;

			if (':' != line[0])
				continue;

			if (!checksum_passes(line + 1))
			{
				fprintf(stderr, "ERROR: bad checksum on line %d of %s; aborting merge\n", lineno, argv[index]);
				return -1;
			}

			count = readhex(line + 1, 2);
			address = readhex(line + 3, 4);
			if ( (0 == strncmp(line + 7, "00", 2)) && ((1 == index) || (0 == upper_address)) ) /* data record */
			{
				blob = find_blob(address, count, (upper_address) ? &config_list : &pm_list);
				ptr = line + 9;
				for (offset = 0; offset < count; offset++)
				{
					blob->data[offset] = readhex(ptr, 2);
					ptr += 2;
				}
			}
			else if (0 == strncmp(line + 7, "04", 2)) /* encoding of upper 16-bits */
			{
				upper_address = readhex(line + 9, 4);
				if (upper_address)
					if (1 != upper_address)
					{
						fprintf(stderr, "ERROR: presumptions about upper address are not valid; aborting\n");
						return -1;
					}
			}
			else if (0 == strncmp(line + 7, "01", 2)) /* end of file record */
			{
				break;
			}
		}

		fclose(handle);
	}

	handle = fopen(argv[index], "wb");

	if (NULL == handle)
	{
		fprintf(stderr, "ERROR: unable to open output file %s\n", argv[index]);
		return -1;
	}

	print_blobs(handle, pm_list);
	fprintf(handle, ":020000040001F9\r\n");
	print_blobs(handle, config_list);
	fprintf(handle, ":00000001FF\r\n");

	fclose(handle);

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

static struct memory_blob *find_blob(unsigned address, unsigned count, struct memory_blob **list)
{
	struct memory_blob *current, *previous, *addition;

	current = *list; previous = NULL;
	while (current)
	{
		if (current->address > address)
			break;

		previous = current;
		current = current->next;
	}

	addition = malloc(sizeof(struct memory_blob));
	memset(addition, 0, sizeof(struct memory_blob));

	addition->data = malloc(count);
	addition->address = address;
	addition->count = count;
	addition->next = current;

	if (previous)
		previous->next = addition;
	else
		*list = addition;

	return addition;
}

static unsigned checksum_passes(const char *text)
{
	unsigned count, checksum, latest;

	latest = readhex(text, 2);
	count = latest + 4;
	checksum = 0;

	while (count--)
	{
		text += 2;
		checksum += latest;
		latest = readhex(text, 2);
	}

	/* Intel Hex checksum is 2's complement of the entire record line */
	checksum = ~checksum + 1;

	return (checksum & 0xFF) == latest;
}

static void print_blobs(FILE *handle, struct memory_blob *blob)
{
	unsigned contiguous_count, checksum, address, linesize, remaining;
	struct memory_blob *seek;
	unsigned char *data;

	while (blob)
	{
		seek = blob; address = blob->address; contiguous_count = 0;

		while (seek)
		{
			if (seek->address != address)
					break;

			address += seek->count; contiguous_count += seek->count;
			seek = seek->next;
		}

		data = blob->data; address = blob->address;
		remaining = blob->count;

		while (contiguous_count)
		{
			linesize = 16 - (address & 0xF);
			if (contiguous_count < linesize)
				linesize = contiguous_count;

			fprintf(handle, ":%02X%04X00", linesize, address);

			checksum  = linesize;
			checksum += (address & 0xFF00) >> 8;
			checksum += (address & 0x00FF) >> 0;

			while (linesize--)
			{
				if (0 == remaining)
				{
					blob = blob->next;
					data = blob->data;
					remaining = blob->count;
				}

				checksum += *data;
				fprintf(handle, "%02X", *data);
				data++; address++; remaining--; contiguous_count--;
			}

			/* Intel Hex checksum is 2's complement of the entire record line */
			checksum = ~checksum + 1;

			fprintf(handle, "%02X\r\n", checksum & 0xFF);

		}

		blob = blob->next;
	}
}
