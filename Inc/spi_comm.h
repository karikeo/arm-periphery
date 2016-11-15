
#define SWAP16(x) ( (x >> 8) | (x << 8) )

#define SPI_RX_SIZE 32
#define SPI_TX_SIZE 32

typedef enum {
  spiCmd_NoCmd = 0,

  spiCmd_FwUpdateInit,
  spiCmd_FwUpdateData,
  spiCmd_FwUpdateComplete,

  spiCmd_ReadLog,

  spiCmd_DexQueryStart,
  spiCmd_DexQueryAbort,
  spiCmd_DexQueryRead,

  spiCmd_SetFbType,

  spiCmd_GetFwVersion,

  spiCmd_GetFbEvent,

  spiCmd_EmuFbEvent,
  
  spiCmd_ReadData,  

  spiCmd_NumCmds
} t_spi_cmd;

typedef enum {
  spiCmd_Ack = 0x8000,
} t_spi_flags;

typedef union {
   struct {
      union {
         u16 w16;
         struct {
            u16 bootloader  : 1;
            u16 error       : 1;
            u16 nda         : 1; //new data available
            u16 log         : 1; //new log data available
            u16 spi_busy    : 1; //
            u16 dex_rdy     : 1; //dex ready
            u16 dex_cmplt   : 1; //dex complete
            u16 ve_inhibit  : 1; //vending inhibited
            u16 fb_m_tmo    : 1; //fieldbus master timeout
            u16 fb_s_tmo    : 1; //fieldbus master timeout
            u16 fb_type     : 2; //fieldbus type
            u16 fb_evt      : 1; //fieldbus evt
            u16 bits_padding: 3;
         } bits;
      } status;
      u16 state;
      u32 param1;
      u32 param2;
      u8 data[ 16 ];
      u8 padding[ 4 ];
      //u8 padding2[ 32 ];
   } fields;

  u8 raw[ 32 ];
} t_spi_in;

typedef union {
   struct {
      union {
         u16 w16;
         struct {
            u16 en_fb_log  : 1;
            u16 en_dex     : 1;
         } bits;
     } control;
     u16 cmd;
     u32 param1;
     u32 param2;
     u8 data[ 16 ];
     u8 padding[ 4 ];
     //u8 padding2[ 8 ];
  } fields ;

  u8 raw[ 32 ];
} t_spi_out;


typedef enum {
  spiState_Idle = 0,

  spiState_FwUpdateInit,
  spiState_FwWrite,
  spiState_FwWrite2,
  spiState_FwWriteComplete,

  spiState_NumStates
} t_spi_mgr_state;

typedef struct {
  u16 state;
} t_spi_mgr;

int spi_mgr( void );

#ifdef __ICCARM__  // IAR
   typedef __task void (*pEntryPoint)(void);
#elif __CC_ARM    // KEIL
   typedef void (*pEntryPoint)(void);
#elif __GNUC__    // GCC
  #warning watch me
#endif
