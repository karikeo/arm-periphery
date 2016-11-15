/* Copyright 2003-2010 IAR Systems AB. */

/*******************
 * This is the default implementation of the "exit" function of the
 * standard library.  It can be replaced with a system-specific
 * implementation.
 *
 * The "exit" function is called when the system performs a normal
 * termination.
 *
 ********************/

#include <stdlib.h>
#include <yfuns.h>

void exit(int arg)
{
  _exit(arg);
}
