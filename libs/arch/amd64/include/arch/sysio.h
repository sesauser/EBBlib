#ifndef _SYSIO_H_
#define _SYSIO_H_
/******************************************************************************
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
*****************************************************************************/

#include <l4hal/types.h>

static inline u32 sysIn32 (u16 port) {
  u32 ret;
  __asm__ volatile ("inl %w1,%0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline u16 sysIn16 (u16 port) {
  u16 ret;
  __asm__ volatile ("inw %w1,%w0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline u8 sysIn8 (u16 port) {
  u16 ret;
  __asm__ volatile ("inb %w1,%b0":"=a"(ret) : "Nd"(port));
  return ret;
}

static inline void sysOut32 (u16 port, u32 val) {
  __asm__ volatile ("outl %0,%w1"::"a"(val), "Nd" (port));
}

static inline void sysOut16 (u16 port, u16 val) {
  __asm__ volatile ("outw %w0,%w1"::"a"(val), "Nd" (port));
}

static inline void sysOut8 (u16 port, u8 val) {
  __asm__ volatile ("outb %b0,%w1"::"a"(val), "Nd" (port));
}

#endif
