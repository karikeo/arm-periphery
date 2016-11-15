#include "stm32f10x.h"
#include "log.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "RTOS.h"


OS_RSEMA SemaLog;

#define SPRINTF_BUF_SIZE 1024
char sprintf_buf[ SPRINTF_BUF_SIZE ];

tLog logDesc;

#if defined (LOG)
////////////////////////////////////////////////////////////////////////////////
void enableLogging() {
   logDesc.prevEnabled = logDesc.enabled;
   logDesc.enabled = 1;
}
////////////////////////////////////////////////////////////////////////////////
void disableLogging() {
   logDesc.prevEnabled = logDesc.enabled;
   logDesc.enabled = 0;
}
////////////////////////////////////////////////////////////////////////////////
u8 isLoggingEnabled() {
   return logDesc.enabled;
}
////////////////////////////////////////////////////////////////////////////////
u8 restoreLogging( void ) {
   return logDesc.enabled = logDesc.prevEnabled;
}
////////////////////////////////////////////////////////////////////////////////
void logInit() {
   int i = 0;
#if 1
   logDesc.head = 0;
   logDesc.head = 0;
#endif
   for ( i = 0; i < LOG_BUFFER_NUMBUFFERS; i++ ) {
      //dbg( " buf#%d) %d \r\n", i, logDesc.buffers[ i ].size );
#if 1
      logDesc.buffers[ i ].size = 0;
#else
      if ( logDesc.buffers[ i ].size > LOG_BUFFER_DATASIZE )
         logDesc.buffers[ i ].size = 0;

#endif

#if !defined (BOOTLOADER)

#else
#endif
   }
}
////////////////////////////////////////////////////////////////////////////////
u32 isLogEmpty() {
   u8 i;
   u32 size = 0;
   for ( i = 0 ; i < LOG_BUFFER_NUMBUFFERS; i++ ) {
      size += logDesc.buffers[ i ].size;
   }
   return size;
}
////////////////////////////////////////////////////////////////////////////////
void logWrite( char* data, int size ) {
   tLogBuffer *b;
   char* p;
   int sizeLeft;
   int chunkSize;

   if ( !logDesc.enabled )
      return;

   sizeLeft = size;

   for ( ; sizeLeft ; ) {

      b = &logDesc.buffers[ logDesc.head ];
      p = &b->data[ b->size ];

      if ( LOG_BUFFER_DATASIZE - b->size < sizeLeft ) {
         chunkSize = LOG_BUFFER_DATASIZE - b->size;
         logDesc.head++;
         logDesc.head = logDesc.head % LOG_BUFFER_NUMBUFFERS;
         logDesc.buffers[ logDesc.head ].size = 0;
         if ( logDesc.head == logDesc.tail )
            logDesc.tail++;
            logDesc.tail = logDesc.tail % LOG_BUFFER_NUMBUFFERS;
      } else {
         chunkSize = sizeLeft;
      }

      memcpy( p, &data[ size - sizeLeft ], chunkSize );
      b->size += chunkSize;
      sizeLeft -= chunkSize;

#ifdef DEBUG
      //if ( sizeLeft )
      //   dbg("\r\nLOG:%d written, h:%d, t:%d, chsz:%d, sizeLeft:%d\r\n\r\n", chunkSize, logDesc.head, logDesc.tail, logDesc.buffers[ logDesc.head ].size, sizeLeft );
#endif
   }

}
////////////////////////////////////////////////////////////////////////////////
int logRead( char* data, int maxSize ) {
   tLogBuffer *b;
   int chunkSize;
   int totalSize = 0;

#if !defined (BOOTLOADER)
   OS_Use( &SemaLog );
#endif

   for ( ; totalSize < maxSize ; ) {

      b = &logDesc.buffers[ logDesc.tail ];
      if ( totalSize + b->size < maxSize )
         chunkSize = b->size;
      else
         chunkSize = maxSize - totalSize ;
#ifdef DEBUG
      //dbg( "tail:%d chunk:%d\r\n", logDesc.tail, chunkSize );
#endif
      memcpy( &data[ totalSize ], b->data, chunkSize );
      totalSize += chunkSize;

      if ( b->size < LOG_BUFFER_DATASIZE ) {
         if ( maxSize < LOG_BUFFER_DATASIZE ) {
            b->size -= chunkSize;
         } else {
            b->size = 0;
         }
         break;
      }

      b->size = 0;
      if ( logDesc.tail == logDesc.head )
         break;

      logDesc.tail++;
      logDesc.tail = logDesc.tail % LOG_BUFFER_NUMBUFFERS;
   }

#if !defined (BOOTLOADER)
   OS_Unuse( &SemaLog );
#endif

   return totalSize;
}
////////////////////////////////////////////////////////////////////////////////
void log1( char* fmt, ...) {
   int rc;
   va_list  list;
   va_start( list, fmt );

#if !defined (BOOTLOADER)
  OS_Use( &SemaLog );
#endif


#if 1
   rc = vsnprintf( sprintf_buf, SPRINTF_BUF_SIZE, fmt, list );
   if ( rc > 0 ) {

//#ifdef DEBUG
#if 0
      int i;
      for ( i = 0; i < rc; ) {
         int n = ( rc - i > 512 ) ? 512 : rc - i;
         //ebdat7_02DebugUartSend( (u8*)&sprintf_buf[ i ], n );
         i += n;
      }
#endif

      if ( logDesc.enabled )
         logWrite( sprintf_buf, rc );
   }
#else
   dbg( fmt, list );
#endif

#if !defined (BOOTLOADER)
  OS_Unuse( &SemaLog );
#endif

   va_end( list );
}
#endif
