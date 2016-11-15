/* Customer-specific DLib configuration. */
/* Copyright (C) 2003 IAR Systems.  All rights reserved. */

#ifndef _DLIB_CONFIG_H
#define _DLIB_CONFIG_H

/*
 * The DLib configuration file controls the features provided by the
 * generated DLib library and the system headers.
 *
 * This is a template configuration file, please update it for your
 * application needs.
 *
 * The original content of this file corresponds to the pre-built
 * "full" configuration.  If no configuration symbols are defined in
 * this file then the library corresponds to the "normal"
 * configuration.
 *
 * For a full description of the configuration symbols, please see the
 * header file DLib_defaults.h.
 */


/* Turn on locale support. */
#define _DLIB_FULL_LOCALE_SUPPORT 1
#define _LOCALE_USE_C

/* Turn on FILE descriptor support. */
#define _DLIB_FILE_DESCRIPTOR 1

/* Turn on multibyte formatting. */
#define _DLIB_FORMATTED_MULTIBYTE 1

#endif /* _DLIB_CONFIG_H */
