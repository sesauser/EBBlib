
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
static int readmem(uintptr_t cookie);
static void writehexbyte(char c);
static int readhexnum(uintptr_t *num, uintptr_t cookie);
static int xdigit_value(int c);

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
    return readmem(cookie);
  default:
    return -1;
  }
}

static int
readmem(uintptr_t cookie) {
  uintptr_t addr, size;
  if(readhexnum(&addr, cookie) != ',')
    return -1;
  if(readhexnum(&size, cookie) != '#')
    return -1;
  printf("+$");
  while(size > 0) {
    writehexbyte(*(char*)addr);
    addr++;
    size--;
  }
  printf("#");
  return 0;
}

static void
writehexbyte(char c) {
  char nibble[2];
  nibble[0] = c & 0x0f;
  nibble[1] = (c & 0xf0) >> 4;
  printf("%x%x", nibble[0], nibble[1]);
}

static int
readhexnum(uintptr_t *num, uintptr_t cookie) {
  int c;
  uintptr_t xdigit;
  *num = 0;
  c = serial_getc(cookie);
  while((xdigit = xdigit_value(c)) != XDIGIT_INVALID) {
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

