#ifndef __MDB_H_
#define __MDB_H_

#define MDB_ACK      0x00
#define MDB_RET      0xAA
#define MDB_NAK      0xFF

#define MDB_MODE_MASK 0x0100

#define MDB_ADR_MASK 0xF8
#define MDB_CMD_MASK 0x07

#define MDB_MAX_LEN  0x24

#define MDB_CMD_CHG_TUBE_STATUS  0x0A
#define MDB_CMD_CHG_COIN_TYPE    0x0C
#define MDB_CMD_CHG_POLL         0x0B
#define MDB_CMD_BV_POLL          0x33
#define MDB_CMD_BV_STACKER       0x36
#define MDB_CMD_BV_BILLTYPE      0x34

#define MDB_ADR_VMC                 0x00 //Reserved for VMC
#define MDB_ADR_CHANGER             0x08 //Changer
#define MDB_ADR_CASHLESS1           0x10 //Cashless Device #1
#define MDB_ADR_GW                  0x18 //Communications Gateway
#define MDB_ADR_DISPLAY             0x20 //Display
#define MDB_ADR_ENERY_MGMT          0x28 //Energy Management System
#define MDB_ADR_BILL_VALIDATOR      0x30 //Bill Validator
#define MDB_ADR_RESERVED            0x38 //Reserved for Future Standard Peripheral
#define MDB_ADR_USD1                0x40 //Universal Satellite Device #1
#define MDB_ADR_USD2                0x48 //Universal Satellite Device #2
#define MDB_ADR_USD3                0x50 //Universal Satellite Device #3
#define MDB_ADR_DISPENSER1          0x58 //Coin Hopper or Tube – Dispenser 1
#define MDB_ADR_CASHLESS2           0x60 //Cashless Device #2
#define MDB_ADR_AGE_DEVICE          0x68 //Age Verification Device
#define MDB_ADR_DISPENSER2          0x70 //Coin Hopper or Tube – Dispenser 2

typedef struct 
{
   u8 rqst_size;
   u16 rqst[ MDB_MAX_LEN ];
   u8 rply_size;
   u16 rply[ MDB_MAX_LEN ];
} t_mdb_session;

typedef struct 
{
   t_mdb_session sessions[ 6 ];
} t_mdb_mon;

typedef enum {
   eFbState_Unknown = 0,
   eFbState_StatusRequest,
   eFbState_StatusReply
} t_fb_state;typedef enum {
   eFbEvtType_Reserved = 0,
   eFbEvtType_FbType,
   eFbEvtType_DoorOpen,
   eFbEvtType_DoorClose,

   eFbEvtType_NumTypes
} t_fb_evt_type;

typedef struct{
   int type;
   int m_tmo      :1;
   int s_tmo      :1;
   int en_log     :1;
   int prev_en_log:1;
   int ve_inhibit :1;
   int door_tr1   :1; 
   u32 emu_evt;

  struct
  {
    u8 enabled : 1;
    
    u32 rx_start_time;
    u32 rx_msg_time;  
    u32 rx_last_time;
    u32 rx_times[ 36 ];
    u8 rx_size;
    u16 rx[ 64 ];
    u8 tx_size;
    u8 tx_cnt;    
    u16 tx[ 64 ];    
  } m;

  struct
  {
    u32 rx_start_time;
    u32 rx_msg_time;  
    u32 rx_last_time;
    
    u8 rx_size;
    u16 rx[ 64 ];
    u8 tx_size;
    u8 tx_cnt;    
    u16 tx[ 64 ];    
  } s;
  
  
  struct 
  {
    u8 enabled : 1;
    u8 display_tr: 1;
    u8 begin_session_tr : 1;
    u8 cancel_session_tr : 1;    
    s32 funds_available;    
    u16 item_price;
    u16 item_no;    
  } cashless;
  
  struct
  {
    u8 power_on : 1;    
    u8 just_reset : 1;
    
  } coin_acceptor;
   
} t_fb_desc;

typedef enum {
   eFbType_Unknown = 0,
   eFbType_EXE,
   eFbType_MDB,
} t_fb_type;




void mdb_evt_s_rx( void );
void mdb_evt_m_rx( void );
void mdb_test( void );

#endif
