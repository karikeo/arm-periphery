/*******************
 *
 * Copyright 1998-2010 IAR Systems AB. 
 *
 * This is the default implementation of the "system" function of the
 * standard library.  It can be replaced with a system-specific
 * implementation.
 *
 * The "system" function tests if the environment has a command
 * processor if is argument i a null pointer -- it returns nonzero if
 * this is the case.  When passed a string it is executed by the
 * command processor and the return value is implementation defined,
 * according to the C standard.  Common UNIX practice, however, is to
 * return -1 on failure.
 *
 * The default implementation returns 0 (i.e. no) when asked if the
 * system has a command processor and -1 (i.e. failure) when called
 * with a command.
 *
 ********************/

#include <stdlib.h>

_STD_BEGIN

#pragma module_name = "?system"

int (system)(const char *s)
{
  return s ? -1 : 0;
}

_STD_END
