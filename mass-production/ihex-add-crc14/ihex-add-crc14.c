/*
    command-line tool to add CRC-14 to PIC16F1454 Intel Hex file
    Copyright (C) 2015,2017 Peter Lawrence

    this utility adds the CRC-14 expected by: 
	https://github.com/majbthrd/PIC16F1-USB-DFU-Bootloader

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

#define PM_SIZE_IN_BYTES                16384
#define	CODE_OFFSET_ADDRESS_IN_BYTES    0x400
#define	HIGH_ENDURANCE_ADDRESS_IN_BYTES 0x3F00

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
static unsigned calc_modified_crc14(unsigned data, unsigned crc);
static void erase_blob(struct memory_blob *blob);

int main(int argc, char *argv[])
{
	FILE *handle;
	char line[256];
	unsigned address, upper_address, offset;
	unsigned count, lineno, crc;
	const char *ptr;
	struct memory_blob *blob, *pm_list;

	pm_list = NULL;

	if (argc < 3)
	{
		fprintf(stderr, "%s <app_ihex> <output_ihex>\n", argv[0]);
		return -1;
	}

	/*
	read in input ihex, discarding config bits
	*/

	handle = fopen(argv[1], "rb");

	if (NULL == handle)
	{
		fprintf(stderr, "ERROR: unable to open input file %s\n", argv[1]);
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
			fprintf(stderr, "ERROR: bad checksum on line %d of %s; aborting merge\n", lineno, argv[1]);
			return -1;
		}

		count = readhex(line + 1, 2);
		address = readhex(line + 3, 4);
		if (0 == strncmp(line + 7, "00", 2) && !upper_address) /* data record */
		{
			blob = find_blob(address, count, &pm_list);
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

	/*
	perform sanity check on data, bailing if it violates allowed conditions
	*/

	blob = pm_list;

	while (blob)
	{
		if ( (blob->address < HIGH_ENDURANCE_ADDRESS_IN_BYTES) && ((blob->address + blob->count) > HIGH_ENDURANCE_ADDRESS_IN_BYTES) )
		{
			fprintf(stderr, "ERROR: CRC address was occupied; app is in conflict with bootloader\n");
			return -1;
		}

		if ( (blob->address < CODE_OFFSET_ADDRESS_IN_BYTES) || ((blob->address + blob->count) > PM_SIZE_IN_BYTES) )
		{
			fprintf(stderr, "ERROR: supplied input file is faulty and used out-of-bounds addresses\n");
			return -2;
		}

		if ( (blob->address & 1) || (blob->count & 1) )
		{
			fprintf(stderr, "ERROR: unexpected and implausible situation where input ihex has an odd number of bytes\n");
			return -3;
		}

		blob = blob->next;
	}

	/*
	fill in unused regions with 0x3FFF (erased state of all bits set) and insert CRC-14
	*/

	address = CODE_OFFSET_ADDRESS_IN_BYTES; crc = 0; blob = pm_list;

	for (;;)
	{
		if (address < blob->address)
		{
			blob = find_blob(address, blob->address - address, &pm_list);
			erase_blob(blob);
		}

		for (offset = 0; offset < blob->count; offset += 2)
		{
			if ((HIGH_ENDURANCE_ADDRESS_IN_BYTES - 2) == address)
			{
				blob->data[offset + 0] = (crc & 0x00FF);
				blob->data[offset + 1] = (crc & 0xFF00) >> 8;
			}
			else
			{
				crc = calc_modified_crc14((unsigned)blob->data[offset + 0] + ((unsigned)blob->data[offset + 1] << 8), crc);
			}
			address += 2;
		}

		address = blob->address + blob->count;

		if (address >= HIGH_ENDURANCE_ADDRESS_IN_BYTES)
			break;

		if (NULL == blob->next)
		{
			blob = find_blob(address, PM_SIZE_IN_BYTES - address, &pm_list);
			erase_blob(blob);
		}
		else
		{
			blob = blob->next;
		}
	}

#if 0
	handle = fopen("c:\\temp\\comp.bin", "wb");

	blob = pm_list;

	while (blob)
	{
		fwrite(blob->data, blob->count, 1, handle);

		blob = blob->next;
	}

	fclose(handle);
#endif

	handle = fopen(argv[2], "wb");

	if (NULL == handle)
	{
		fprintf(stderr, "ERROR: unable to open output file %s\n", argv[2]);
		return -1;
	}

	print_blobs(handle, pm_list);
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

static unsigned calc_modified_crc14(unsigned data, unsigned crc)
{
	unsigned bit, result;

	/*
	this is a modified CRC-14:
	it shifts in 16 (rather than 14) bits
	*/

	for (bit = 0; bit < 16; bit++)
	{
		result = (data & 0x0001) ^ (crc & 0x0001);
		crc >>= 1;
		if (result)
			crc ^= 0x23B1;
		data >>= 1;
	}

	return crc;
}

static void erase_blob(struct memory_blob *blob)
{
	unsigned char *data = blob->data;
	unsigned count = blob->count;

	while (count)
	{
		*data++ = 0xFF;	*data++ = 0x3F;
		count -= 2;
	}
}
