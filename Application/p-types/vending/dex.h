#define NUL 0x00
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define EOT 0x04
#define ENQ 0x05
#define ACK 0x06
#define DLE 0x10
#define NAK 0x15
#define SYN 0x16
#define ETB 0x17

typedef enum
{
  dexState_Initial = 0,
  dexState_Idle,
  dexState_SendENQ,
  dexState_CheckENQ,
  dexState_SlaveHandshake1,
  dexState_SlaveHandshake2,
  dexState_SlaveHandshake3,
  dexState_MasterHandshake1,
  dexState_MasterHandshake2,
  dexState_ThirdHandshake,
  dexState_SendAuditData,
  dexState_RecvAuditData1,
  dexState_RecvAuditData2,
  dexState_RecvAuditData3,
} eDexState;

typedef enum
{
   dexResult_InProgress,
   dexResult_Ok,
   dexResult_Timeout,
   dexResult_ExtReaderPlugged,
} eDexResult;

#define AUDIT_DATA_BUFFER_SIZE 5 * 1024

typedef struct 
{
   u8 enable;
   u16 state;
   OS_TIMER timer;

   u8 bMasterHSDone     :1;
   u8 bSlaveHSDone      :1;
   u16 crc;
   u16 tx_cnt;
   u16 tx_size;
   u8 tx_buf[ 64 ];
   u16 rx_size;
   u8 rx_buf[1 * 1024];

   u8 audit_data_valid : 1;
   u16 audit_size;
   u16 audit_ptr;
   u8 audit_data[ AUDIT_DATA_BUFFER_SIZE ];

   int numBlocksReceived;
   int blockDataSize;
   u8 bDleToggle        :1;

   union {
      struct {
         u16 query_cmplt : 1;
         u16 query_error : 1;
      } bits;
      u16 raw;
   } triggers;

   u8 sema : 1;

   int result;
   //t_emb_SysTimer dexResultTime;
   char resultDescr[ 128 ];

 } tDex;

void dex_init(void);
int read_dex(void);
void start_dex_query( void );
int abort_dex_query( void );
void calc_crc2( char* uData, u16 size);
u16 calc_crc3( char* uData, int size);
void calc_crc(u16 *pCrc, u8 uData);
void dexTx(void);
void dexStartTimer( u32 timeout );
void dexCancelTimer(void);
void gotoDexState( int state, unsigned long timeout_ms );
void printDexBytes( char* prefix, u8* buf, int size );
void dex_rx_isr(u8 b);
u16 dex_tx_isr(u8* b);
void dex_isr( void* uart );
