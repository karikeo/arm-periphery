/*******************
 *
 * Copyright 2006-2014 IAR Systems. All rights reserved.
 *
 * This is a template implementation of the routine responsible
 * for reporting a failed assert.
 *
 * To use a customized assert reporting mechanism, adjust a copy of 
 * this file to your needs and include it in the project.
 *
 ********************/
 
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#pragma language=extended

__interwork void __aeabi_assert(const char * mess, const char * file, int line)
{	
	/* print assertion message and abort */

	printf("%s:%d %s -- assertion failed\n", file, line, mess);
	abort();
}
  
