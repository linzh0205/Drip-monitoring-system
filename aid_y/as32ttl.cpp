/*
 *  AS32-TTL-100 library
 */

#include <Arduino.h>
#include "as32ttl.h"

as32ttl::as32ttl(){  
  CFG.HEAD=0xC0;
  CFG.SPED=0x1A;
  CFG.OPTION=0xC0;
  mode=0;
}

as32ttl::~as32ttl(){
  
}

void as32ttl::begin(uint16_t address, uint8_t channel, byte MD0pin, byte MD1pin, byte AUXpin){
  md0=MD0pin;
  md1=MD1pin;
  aux=AUXpin;
  pinMode(md0,OUTPUT);
  pinMode(md1,OUTPUT);
  pinMode(aux,INPUT);
  digitalWrite(md0,LOW);
  digitalWrite(md1,LOW);
  mode=0;

  Serial.begin(9600);
  
  getParameterSetting(&CFG);
  CFG.HEAD=0xC0;
  CFG.ADDH=uint8_t (address>>8);
  CFG.ADDL=uint8_t (address&0x00ff);
  CFG.CHAN=channel;
  CFG.SPED=0x1A;
  CFG.OPTION=0xC0;
  setParameterSetting(&CFG);  
}

void as32ttl::begin(byte MD0pin, byte MD1pin, byte AUXpin){
  md0=MD0pin;
  md1=MD1pin;
  aux=AUXpin;
  pinMode(md0,OUTPUT);
  pinMode(md1,OUTPUT);
  pinMode(aux,INPUT);
  digitalWrite(md0,LOW);
  digitalWrite(md1,LOW);
  mode=0;
  Serial.begin(9600);
  getParameterSetting(&CFG);
}

int as32ttl::setRecieverAddressChannel(uint16_t address, uint8_t channel){
  return setRecieverAddressChannel(uint8_t (address>>8), uint8_t (address&0x00ff), uint8_t (channel));
}

int as32ttl::setRecieverAddressChannel(uint8_t addrH, uint8_t addrL, uint8_t chnl){
  CFG.HEAD=0xC0;
  CFG.ADDH=addrH;
  CFG.ADDL=addrL;
  CFG.CHAN=chnl;
  CFG.SPED=0x1A;
  CFG.OPTION=0xC0;

  return setParameterSetting(&CFG);  
}

uint16_t  as32ttl::getRecieverAddress(){
  
  return uint16_t (((CFG.ADDH<<8)&0xff00) | (CFG.ADDL & 0x00ff));
}

uint8_t  as32ttl::getRecieverAddressH(){
  
  return CFG.ADDH;
}

uint8_t  as32ttl::getRecieverAddressL(){
  
  return CFG.ADDL;
}

uint8_t as32ttl::getRecieverChannel(){
  return CFG.CHAN;
}

void as32ttl::cleanSerialBuf(){
  while (Serial.available())
  {
    Serial.read();
  }  
}

int as32ttl::chkAUXH(int maxtims){
  int cntTime=0;
  while (digitalRead(aux)==LOW) {
    delay(10);
    if (cntTime > maxtims) return 1;
    cntTime++;
  }
  return 0;
}

int as32ttl::setMormalMode(){       // set module to normal mode, return 0 secuessed ,else return 1 is timeout
  if (mode==0) return 0;            // older mode is normal, nothing to do
  if (chkAUXH(modeChangeTimeOut)) return 1;
  digitalWrite(md0,LOW);
  digitalWrite(md1,LOW);
  mode=0;
  delay(modeChangeReadyTime);
  if (chkAUXH(modeChangeTimeOut)) return 1;
  return 0;
}

int as32ttl::setWakeUpMode(){       // set module to wakeup  mode, return 0 successed ,else return 1 is timeout
  if (mode==1) return 0;            // older mode is WakeUp, nothing to do 
  if (chkAUXH(modeChangeTimeOut)) return 1;
  digitalWrite(md0,HIGH);
  digitalWrite(md1,LOW);
  mode=1;
  delay(modeChangeReadyTime);
  if (chkAUXH(modeChangeTimeOut)) return 1;
  return 0;
}

int as32ttl::setPowerSavingMode(){    // set module to power saving mode, return 0 successed ,else return 1 is timeout
  if (mode==2) return 0;
  if (chkAUXH(modeChangeTimeOut)) return 1;
  digitalWrite(md0,LOW);
  digitalWrite(md1,HIGH);
  mode=2;
  delay(modeChangeReadyTime);
  if (chkAUXH(modeChangeTimeOut)) return 1;
  return 0;  
}

int as32ttl::setSleepMode(){        // set module to sleep mode, return 0 successed ,else return 1 is timeout
  if (mode==3) return 0;
  if (chkAUXH(modeChangeTimeOut)) return 1;
  digitalWrite(md0,HIGH);
  digitalWrite(md1,HIGH);
  mode=3;
  delay(modeChangeReadyTime);
  if (chkAUXH(modeChangeTimeOut)) return 1;
  return 0;    
}

int as32ttl::setParameterSetting(struct configStruct *CFG){   // return 0 successed ,else return 1 is timeout, return 2 is "not return old mode"
  if (setSleepMode()) return 1;                  //set command fail
  if (chkAUXH(modeChangeTimeOut)) return 1;
  Serial.write((uint8_t *)CFG,6);                //write new config parameter
  delay(cmdReturnTime);
  int rcvBytes = Serial.available();
  //--------- for debug -------
  //if (rcvBytes<1) led13f(3);
  //----------------------------
  if (rcvBytes==4){
      char buf[4];
      Serial.readBytes(buf, rcvBytes);
  }else cleanSerialBuf();

  return 0;                                     // successed return zero
}

int as32ttl::getParameterSetting(struct configStruct *CFG){   // return 0 successed ,else return 1 is timeout, return 2 is "not return old mode", return 3 is read error
  uint8_t cmd[3]={0xc1,0xc1,0xc1};              // read config parameter
  if (setSleepMode()) return 1;                 // set command mode fail
  if (chkAUXH(modeChangeTimeOut)) return 1;
  cleanSerialBuf();
  Serial.write(cmd,3);
  delay(cmdReturnTime);
  int byteCnt=Serial.available();
  if (byteCnt==sizeof(struct configStruct)){
    Serial.readBytes((char *)CFG,&byteCnt);
  }else return 3;                               // config parameter read fail
  return 0;                                     // successed return 0
}

int as32ttl::getMode(){
  /*
  int m0,m1;
  m0=digitalRead(md0);
  m1=digitalRead(md1);
  return (((m1&0x1)<<1) | (m0&0x1));
  */
  return mode;
}

int as32ttl::getFirmwareVer(char *Fver){   // return 0 successed, else return 1 is timeout,  return 2 is "not return old mode", return 3 is read error
  uint8_t cmd[3]={0xc3,0xc3,0xc3};         // get firmware version
  if (setSleepMode()) return 1;
  cleanSerialBuf();                        // clean serial port buffer 
  Serial.write((uint8_t *)cmd,3);
  delay(cmdReturnTime);                    // wait for module return
  int byteCnt=Serial.available();
  if (byteCnt > 0 ){
    Serial.readBytes(Fver,byteCnt);
    //for(int i=0;i<byteCnt;i++)
    //  *(Fver+i)=Serial.read();  
  }else return 3;
  return 0;  
}

int as32ttl::resetModule(){               // return 0 successed ,else return 1 is timeout
  uint8_t cmd[3]={0xc4,0xc4,0xc4};        // reset module
  if (setSleepMode()) return 1;  
  Serial.write((uint8_t *)cmd,3); 
  delay(cmdReturnTime);
  return 0;  
}

uint16_t as32ttl::getVoltage(){
  uint8_t cmd[3]={0xc5,0xc5,0xc5};        // get module power supply voltage
  if (setSleepMode()) return 1;  
  Serial.write((uint8_t *)cmd,3); 
  delay(cmdReturnTime);
  int byteCnt=Serial.available();
  uint16_t v;
  if (byteCnt==2) {
    v=((uint16_t) Serial.read()) << 8;
    v=(v & 0x00ff) | ((uint16_t) Serial.read());
  }
  return v; 
}

int as32ttl::available(){            // LORA transfers some data to Serial and available to read
  return Serial.available();
}

uint8_t as32ttl::get(){              // read a byte
  return (uint8_t ) Serial.read();
}

int as32ttl::gets(uint8_t* buf){     // read a lot of bytes, return length of data
  int dataCnt=Serial.available();
  if (dataCnt>0) for(int i=0;i<dataCnt;i++) *(buf+i)=(uint8_t ) Serial.read();
  return dataCnt;
}

int as32ttl::availableForWrite(){   // LORA & Serial available to write
  return Serial.availableForWrite();
}

int as32ttl::send(uint16_t address,uint8_t channel,uint8_t a){  // send a char to address:channel
  if (mode!=0) {if (setMormalMode()) return 3;}
  
  uint8_t buf[4];
  buf[0]=uint8_t ((address & 0xff00)>>8);
  buf[1]=uint8_t (address & 0xff);
  buf[2]=channel;
  buf[3]=a;
  
  int cntTime=0;
  int bytesForWrite=availableForWrite();
  if (bytesForWrite<4){
    while ( (bytesForWrite=availableForWrite()) < 4){
      delay(1);
      if (cntTime > waitWriteSerialTimeOut) return 1;
      cntTime++;
    }
  }
  if (chkAUXH(waitForAuxHighTimeOut)) return 2;
  Serial.write(buf,4);
  return 0;
}

int as32ttl::send(uint16_t address,uint8_t channel,char* buf){  // send a string to address:channel
  if (mode!=0) {if (setMormalMode()) return 3;}
  uint8_t hbuf[3];
  int len;
  hbuf[0]=uint8_t ((address & 0xff00)>>8);
  hbuf[1]=uint8_t (address & 0xff);
  hbuf[2]=channel;
  len = strlen(buf);
  
  int cntTime=0;
  int bytesForWrite=availableForWrite();
  if (bytesForWrite<len){
    while ( (bytesForWrite=availableForWrite()) < len){
      delay(1);
      if (cntTime > waitWriteSerialTimeOut) return 1;
      cntTime++;
    }
  }
  if (chkAUXH(waitForAuxHighTimeOut)) return 2;
  Serial.write(hbuf,3);
  Serial.write(buf,len);

  return 0;
}

int as32ttl::send(uint16_t address,uint8_t channel,uint8_t* buf,int len){   // send number data 
  if (mode!=0) {if (setMormalMode()) return 3;}
  uint8_t hbuf[3];
  hbuf[0]=uint8_t ((address & 0xff00)>>8);
  hbuf[1]=uint8_t (address & 0xff);
  hbuf[2]=channel;
  
  int cntTime=0;
  int bytesForWrite=availableForWrite();
  if (bytesForWrite<len){
    while ( (bytesForWrite=availableForWrite()) < len){
      delay(1);
      if (cntTime > waitWriteSerialTimeOut) return 1;
      cntTime++;
    }
  }
  if (chkAUXH(waitForAuxHighTimeOut)) return 2;
  Serial.write(hbuf,3); 
  Serial.write(buf,len);

  return 0;
}

int as32ttl::sendsc(uint16_t address,uint8_t a){  // send a char to address:channel
  return send(address,CFG.CHAN,a);
}

int as32ttl::sendsc(uint16_t address,char* buf){  // send a string to address:channel
  return send(address,CFG.CHAN,buf);
}

int as32ttl::sendsc(uint16_t address,uint8_t* buf,int len){   // send number data 
  return send(address,CFG.CHAN,buf,len);
}
