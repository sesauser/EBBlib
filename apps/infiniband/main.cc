/* Copyright 2011 Boston University. All rights reserved. */

/* Redistribution and use in source and binary forms, with or without modification, are */
/* permitted provided that the following conditions are met: */

/*    1. Redistributions of source code must retain the above copyright notice, this list of */
/*       conditions and the following disclaimer. */

/*    2. Redistributions in binary form must reproduce the above copyright notice, this list */
/*       of conditions and the following disclaimer in the documentation and/or other materials */
/*       provided with the distribution. */

/* THIS SOFTWARE IS PROVIDED BY BOSTON UNIVERSITY ``AS IS'' AND ANY EXPRESS OR IMPLIED */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND */
/* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BOSTON UNIVERSITY OR */
/* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR */
/* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF */
/* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/* The views and conclusions contained in the software and documentation are those of the */
/* authors and should not be interpreted as representing official policies, either expressed */
/* or implied, of Boston University */


#include <l4io.h>
#include <l4/kdebug.h>
#include <l4/ipc.h>

#include <acpi.h>
#include <pcie.h>


int main (void)
{
	printf("Infiniband Driver started\n");
	acpi_init();
	printf("ACPI initialized\n");
	pcie_init();
	printf("PCIe initialized\n");
//	infiniband_init();
//	printf("Infiniband initialized\n");
	int choice = 0;
	do
	{
		printf("(C)lient or (S)erver:\n");
		choice = getc();
		if (choice=='C') choice = 'c';
		if (choice=='S') choice = 's';
	}
	while (choice!='c' && choice!='s');
	if (choice == 'c')
	{	// client mode
		printf("start sending data...\n");
	} else
	{	// server moce
		printf("listening for data\n");
	}
	for (;;)
		L4_KDB_Enter("EOW");
	return 0;
}
