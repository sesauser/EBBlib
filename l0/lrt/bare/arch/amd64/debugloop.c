
/*
 * Copyright (C) 2011 by Project SESA, Boston University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "debugloop.h"
#include "serial.h"

static const int XDIGIT_INVALID = 16;

static int handle_gdbreq(uintptr_t cookie);
static int readmem(uintptr_t cookie, uint8_t checksum);
static void writehexbyte(uint8_t c, uint8_t *checksum);
static int readhexnum(uintptr_t *num, uintptr_t cookie, uint8_t *checksum);
static int xdigit_value(int c);
static int writemem(uintptr_t cookie, uint8_t checksum);

void
debugloop(uintptr_t cookie) {
  while(1) {
    if(handle_gdbreq(cookie))
      printf("-");

  }
}

static int
handle_gdbreq(uintptr_t cookie) {
  uint8_t c = serial_getc(cookie);
  if(c != '$')
    return -1;
  switch(c = serial_getc(cookie)) {
  case 'm':
    return readmem(cookie, 'm');
  case 'M':
    return writemem(cookie, 'M');
  default:
    return -1;
  }
}

static int
readmem(uintptr_t cookie, uint8_t checksum) {
  uintptr_t addr, size;
  uint8_t reply_checksum = 0;
  if(readhexnum(&addr, cookie, &checksum) != ',')
    return -1;
  checksum += ',';
  if(readhexnum(&size, cookie, &checksum) != '#')
    return -1;
  // TODO: we might want to actually check the checksum here,
  // I don't currently because I haven't hooked things up to gdb yet,
  // and don't want to calculate it. We're still reading it in though. --isd
  serial_getc(cookie);
  serial_getc(cookie);
  printf("+$");
  while(size > 0) {
    writehexbyte(*(uint8_t*)addr, &reply_checksum);
    addr++;
    size--;
  }
  printf("#");
  writehexbyte(reply_checksum, NULL);
  return 0;
}

static int
writemem(uintptr_t cookie, uint8_t checksum) {
  uintptr_t addr, size;
  uint8_t c;
  uint8_t reply_checksum;
  if(readhexnum(&addr, cookie, &checksum) != ',')
    return -1;
  checksum += ',';
  if(readhexnum(&size, cookie, &checksum) != ':')
    return -1;
  c = serial_getc(cookie);
  while(c != '#' && size > 0) {
    uint8_t byte;
    byte = xdigit_value(c);
    byte |= xdigit_value(serial_getc(cookie)) * 16;
    *(uint8_t*)addr = byte;
    addr++;
    size--;
    c = serial_getc(cookie);
  }
  // get the checksum. currently not actually checking it.
  serial_getc(cookie);
  serial_getc(cookie);
  reply_checksum = 'O' + 'K';
  printf("+$OK#");
  writehexbyte(reply_checksum, NULL);
  return 0;
}

static void
writehexbyte(uint8_t c, uint8_t *checksum) {
  uint8_t nibble[2];
  nibble[0] = c & 0x0f;
  nibble[1] = (c & 0xf0) >> 4;
  printf("%x%x", nibble[0], nibble[1]);
  if(checksum)
    *checksum += nibble[0] + nibble[1];
}

static int
readhexnum(uintptr_t *num, uintptr_t cookie, uint8_t *checksum) {
  int c;
  uintptr_t xdigit;
  *num = 0;
  c = serial_getc(cookie);
  while((xdigit = xdigit_value(c)) != XDIGIT_INVALID) {
    *checksum += c;
    *num *= 16;
    *num += xdigit;
    c = serial_getc(cookie);
  }
  return c;
}

static int
xdigit_value(int c) {
  if(c <= '9' && c >= '0')
    return c - '0';
  if(c <= 'f' && c >= 'a')
    return c - 'a' + 10;
  if(c <= 'F' && c >= 'A')
    return c - 'A' + 10;
  return XDIGIT_INVALID;
}

