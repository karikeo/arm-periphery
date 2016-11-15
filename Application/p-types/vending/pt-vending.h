//#define USART3_DEX
//#define DEX_SAMPLE

//#define DEX USART3
#define MDB_SNIFFER USART3
//#define MDB_COIN_CHANGER
#define MDB_CASHLESS1
//#define MDB_CASHLESS2
//#define MDB_PROXY BILL_VALIDATOR

#define SW_UART

#define EVT_FB_EMU              (1<<0)
#define EVT_DEX_DRD             (1<<1)
#define EVT_DEX_START           (1<<4)
#define EVT_DEX_ABORT           (1<<5)
#define EVT_FB_M_RX             (1<<2)
#define EVT_FB_S_RX             (1<<3)
#define EVT_FB_M_RX_TMO         (1<<6)
#define EVT_FB_S_RX_TMO         (1<<7)
#define EVT_SNIFF_RX_MSG        (1<<8)
  
#define EVT_CL_VEND_REQUEST     (1<<16)
#define EVT_CL_VEND_OK          (1<<17)
#define EVT_CL_VEND_CANCEL      (1<<18)
#define EVT_CL_VEND_FAIL        (1<<19)
#define EVT_CL_SESSION_COMPLETE (1<<20)

#define ARRAY_SIZE(x) sizeof(x) / sizeof(x[0])

typedef struct
{
  u16 start_dex         :1;
  u16 abort_dex         :1;
  u16 read_dex          :1;
  u16 cmd;
} pt_vending_in;

typedef struct
{
  u16 status;
  u8 cmd;
  u8 rc;
  u16 ptr;
  u16 size;
  u16 data[ 8 ];
} pt_vending_out;



typedef struct
{
   int h;
   int t;
   int max_size;
   int size;
   int item_size;
   void* items;
} t_q;

typedef struct
{
   int type;
} t_fb_evt;

__irq void USART1_IRQHandler(void);
__irq void USART2_IRQHandler(void);
__irq void USART3_IRQHandler(void);

void init_uarts( void );
void Task_vending( void );

void timer1000_cb( void );
void fbMasterTimeout1( void );
void fbSlaveTimeout1( void );
void fbm_msg_end( void );
void fbs_msg_end( void );
void init_fb( void );
void fb_s_tx( u16*, u16 );
void fb_m_tx( u16*, u16 );
void detectFbType( u16 b );

void Task_USART3(void);
void USART3_Timeout( void );
