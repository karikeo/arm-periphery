/*******************
 *
 * Copyright 1998-2010 IAR Systems AB. 
 *
 * This is the default implementation of the "raise" function of the
 * standard library.  It can be replaced with a system-specific
 * implementation.
 *
 * The "raise" function should call the signal handler for the raised
 * signal, or terminate and possibly report an error.
 *
 ********************/

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
_STD_BEGIN

int (raise)(int sig)
{	/* raise a signal */
  _Sigfun *const s = signal(sig, (_Sigfun *)SIG_IGN);

  if (s == (_Sigfun *)SIG_ERR)
    return (-1);	/* bad signal */
  else if (s == (_Sigfun *)SIG_IGN)
    ;
  else if (s != (_Sigfun *)SIG_DFL)
  {	/* revert and call handler */
    signal(sig, (_Sigfun *)SIG_DFL);
    (*s)(sig);
  }
  else
  {	/* default handling */
    exit(EXIT_FAILURE);
  }
  return 0;
}

_STD_END

/*
 * Copyright (c) 1992-2009 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V5.04:0576 */
