#define LOG

#define LOG_BUFFER_DATASIZE 16

typedef struct {
   char data[ LOG_BUFFER_DATASIZE ];
   int size;
} tLogBuffer;

#define LOG_BUFFER_NUMBUFFERS 128
typedef struct {
   tLogBuffer buffers[ LOG_BUFFER_NUMBUFFERS ];
   int head;
   int tail;
   u8 enabled:1;
   u8 prevEnabled:1;
} tLog;

#if defined (LOG)
void log1( char* fmt, ...);
#define log( x, ARGS...) log1( x, ##ARGS )
extern tLog logDesc;
void enableLogging( void );
void disableLogging( void );
u8 isLoggingEnabled( void );
u8 restoreLogging( void );
void logInit( void );
int logRead( char* data, int maxSize );
void logWrite( char* data, int size );
u32 isLogEmpty( void );
#else
#define log( x, ARGS...) dbg( x, ##ARGS )
#define enableLogging()
#define disableLogging()
#define logInit(x)
#define logRead
#define logWrite
#endif

#if 1
#define dbg( x... ) goto_cursor(0x10); lcd_print( x );

#else
#define dbg( x... )
#endif


#define O(c)            (c) - 0x19
#define UNHIDE_STR(str) do { char *p = str;  while (*p) *p++ += 0x19; } while (0)
#define HIDE_STR(str)   do { char *p = str;  while (*p) *p++ -= 0x19; } while (0)
