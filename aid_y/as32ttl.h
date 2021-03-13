#ifndef as32ttl_h
#define as32ttl_h


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// define speed byte format

// define UART format
#define uartFormat8N1 0x00
#define uartFormat8O1 0x40
#define uartFormat8E1 0x80
#define uartFormatMask 0x3f

// define UART bps
#define uart1200bps   0x00
#define uart2400bps   0x08
#define uart4800bps   0x10
#define uart9600bps   0x18
#define uart19200bps  0x20
#define uart38400bps  0x28
#define uart57600bps  0x30
#define uart115200bps 0x38
#define uartBpsMask   0xc7

// define air data rate (bps)
#define airDataRate0p3kBps  0x00
#define airDataRate1p2kBps  0x01
#define airDataRate2p4kBps  0x02
#define airDataRate4p8kBps  0x03
#define airDataRate9p6kBps  0x04
#define airDataRate19p2kBps 0x05
#define airDataRateBpsMask  0xf8

// define frequency channnel
#define channel_410MHz 0x00
#define channel_411MHz 0x01
#define channel_412MHz 0x02
#define channel_413MHz 0x03
#define channel_414MHz 0x04
#define channel_415MHz 0x05
#define channel_416MHz 0x06
#define channel_417MHz 0x07
#define channel_418MHz 0x08
#define channel_419MHz 0x09
#define channel_420MHz 0x0a
#define channel_421MHz 0x0b
#define channel_422MHz 0x0c
#define channel_423MHz 0x0d
#define channel_424MHz 0x0e
#define channel_425MHz 0x0f
#define channel_426MHz 0x10
#define channel_427MHz 0x11
#define channel_428MHz 0x12
#define channel_429MHz 0x13
#define channel_430MHz 0x14
#define channel_431MHz 0x15
#define channel_432MHz 0x16
#define channel_433MHz 0x17
#define channel_434MHz 0x18
#define channel_435MHz 0x19
#define channel_436MHz 0x1a
#define channel_437MHz 0x1b
#define channel_438MHz 0x1c
#define channel_439MHz 0x1d
#define channel_440MHz 0x1e
#define channel_441MHz 0x1f

// define OPTION Byte
#define transparentTransmissionMode 0x00
#define fixedTransmissionMode       0x80

#define TXD_AUX_OC_RXD_OC   0x00
#define TXD_AUX_PP_RXD_PU   0x40    // default

#define RFWakeUpTime250ms   0x00    // default
#define RFWakeUpTime500ms   0x08
#define RFWakeUpTime750ms   0x10
#define RFWakeUpTime1000ms  0x18
#define RFWakeUpTime1250ms  0x20
#define RFWakeUpTime1500ms  0x28
#define RFWakeUpTime1750ms  0x30
#define RFwakeUpTime2000ms  0x38

#define turnOffFEC  0x00
#define turnOnFEC   0x04  // default

#define transmissionPower20dBm  0x00
#define transmissionPower17dBm  0x01
#define transmissionPower14dBm  0x02
#define transmissionPower10dBm  0x03

#define modeChangeTimeOut 100       //10ms 100 times
#define waitWriteSerialTimeOut 10   //1ms 10 times
#define waitForAuxHighTimeOut 100   //10ms 100 times
#define modeChangeReadyTime 500     // ms
#define cmdReturnTime 200           // ms
#define rcvAddrChnlTime 100         //2400 ms 50 times


struct configStruct {
  uint8_t HEAD;             // message head, C0:Save the parameters when power-down or C2:Not save the parameters when power-down
  uint8_t ADDH;             // module address high byte
  uint8_t ADDL;             // module address low byte
  uint8_t SPED;             // RF air speed : uart bps : uart format
  uint8_t CHAN;             // RF Communication frequency（410M + CHAN * 1M）Default 17H（433MHz）
  uint8_t OPTION;           // RF option
};

class as32ttl
{
   private:
    //pin define
    uint8_t md0,md1,aux;
    struct configStruct CFG; // config data
    uint8_t mode;
    
  public:
    as32ttl();
    virtual ~as32ttl();     
    
    void begin(uint16_t address, uint8_t channel, byte MD0pin, byte MD1pin, byte AUXpin);
    void begin(byte MD0pin, byte MD1pin, byte AUXpin);
    
    int setRecieverAddressChannel(uint16_t address, uint8_t channel);             // return 0 successed ,else return 1 is timeout
    int setRecieverAddressChannel(uint8_t addrH, uint8_t addrL, uint8_t chnl);    // return 0 successed ,else return 1 is timeout
      
    uint16_t getRecieverAddress();      // get reciever LORA 16 bits adress
    uint8_t getRecieverAddressH();
    uint8_t getRecieverAddressL();
    uint8_t getRecieverChannel();       // get reciever LORA 8 bits channel number

    void cleanSerialBuf();              // empty recieve serial buffer
    
    int setMormalMode();       // set module to normal mode, return 0 successed ,else return 1 is timeout
    int setWakeUpMode();       // set module to wakeup  mode, return 0 successed ,else return 1 is timeout
    int setPowerSavingMode();  // set module to power saving mode, return 0 successed ,else return 1 is timeout
    int setSleepMode();        // set module to sleep mode, return 0 successed ,else return 1 is timeout
    int setParameterSetting(struct configStruct *CFG);   // return 0 successed ,else return 1 is timeout
    int getParameterSetting(struct configStruct *CFG);   // return 0 successed ,else return 1 is timeout
    int getMode();
    int getFirmwareVer(char *Fver);  // return 0 successed, else return 1 is timeout
    int resetModule();        // return 0 successed ,else return 1 is timeout
    uint16_t getVoltage();
    int chkAUXH(int maxtims);

    int available();            // LORA transfers some data to Serial and available to read
    uint8_t get();              // read a byte
    int gets(uint8_t* buf);     // read a lot of bytes, return length of data
    int availableForWrite();    // LORA & Serial available to write
    int send(uint16_t address,uint8_t channel,uint8_t a);  // send a char to address:channel
    int send(uint16_t address,uint8_t channel,char* buf);  // send a string to address:channel
    int send(uint16_t address,uint8_t channel,uint8_t* buf,int len); // send number data 
    int sendsc(uint16_t address,uint8_t a);           // send a char to address with same channel
    int sendsc(uint16_t address,char* buf);           // send a string to address with same channel
    int sendsc(uint16_t address,uint8_t* buf,int len);// send number data
      
};

#endif
