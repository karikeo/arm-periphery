Readme file for C/C++ library project
=====================================

This project builds an archive consisting of the C and C++ parts of the library 
for EWARM (math library functions are excluded).

To customize the library:
 * set up your favorite options: make sure the output goes where you want it,
 * edit dlArmCustom.h to suit your needs, and
 * click make.

   If you need to adapt the library's I/O functionality to specific hardware,
   there is no need to rebuild a whole new library. Just include the files
   found in <installdirectory>\arm\src\lib in your application project and they 
   will be used instead of the standard ones in the library.
   For more information on these files, see the chapter The DLIB runtime 
   environment in the IAR C/C++ Development Guide for ARM.

   You must be very careful if changing the library code, it is very easy 
   to modify the library code such that the compiler neither provides "working" 
   code nor conforms to the ISO/ANSI standard. Even if you have a "good compiler 
   users" knowledge of the ISO/ANSI standard, you may inadvertently make such 
   changes. 

   While we may be able to assist you with problems after you have modified library 
   code, making library code changes implies that you take on responsibility for the 
   behavior of the compiler and its environment.
                   

The project consists of two groups of source files:

   Beware that some of these non-standard library functions are tightly connected 
   to the compiler. Changes in library code can lead not only to problems during 
   building, but also to problems that show up at run-time. Such problems may be 
   intermittent and very hard to find.   

 * Generic DLIB. This group contains the C, (E)EC++ and C++ parts of the library.
   Functions like atoi() and qsort() are found here. More complex functionality, 
   like formatted I/O, is divided into several files.

 * Low-level interface. Includes local copies of the low-level interface to the library.
   Use and modify these files if you want to use file I/O (or direct non-file I/O),
   implement system, time or signal.
 
 Besides these groups, there is a local copy of a configuration header for the library.
 Edit this file to enable/disable features like file I/O and Locale. Remember to use this
 file as configuration when building applications with the generated library.