typedef struct 
{
  OS_TIMER timer;
  u16 tx_cnt;
  u16 tx_size;
  u8 tx_buf[ 256 ];
  u16 rx_size;
  u8 rx_buf[ 32 ];

  struct 
  {
    struct
    {
      u16 rx[ MDB_MAX_LEN ];
      u8 rx_size;
      u32 time;
    } raw;
  } m;

  struct 
  {
    struct 
    {
      u16 rx[ MDB_MAX_LEN ];
      u8 rx_size;
      u32 time;      
    } raw;
  } s;

  u8 sema : 1;

} tMdbSniffer;

#define SNIFFER_MASTER_CH 'M'
#define SNIFFER_SLAVE_CH 'S'

void sniffer_isr( void* uart );
void sniffer_rx_msg( void );
void sniffer_m_rx( void );
void sniffer_s_rx( void );
void sniffer_tx( u8* p, u16 n );

