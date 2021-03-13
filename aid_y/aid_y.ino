#include <avr/sleep.h>
#include <PinChangeInt.h>
#include <Timers.h>

#include "HX711.h"
#include "as32ttl.h"
#include <EEPROM.h>
#include "SoftwareReset.h"

#define onbd_led 13
#define red_led 2     //STOP狀態LED紅燈號腳位
#define green_led 5   //Start狀態LED綠燈號腳位
#define blue_led 10   //電量狀態LED blue燈號腳位 
#define rf_md0 6      //lora as32-ttl-100 mode pin 0
#define rf_md1 7      //lora as32-ttl-100 mode pin 1
#define rf_aux 4      //lora as32-ttl-100 aux pin
#define hx711_DT 11   //HX711 Data output
#define hx711_PD_SCK 12 //HX711 PD_SCK clock & power down controll pin
#define start_bt 9    //start button switch pin 
#define stop_bt 8     //stop button switch pin
#define batteryVoltageIn  A0
#define batteryScale 0.034042553

#define ledOn digitalWrite(onbd_led,HIGH)
#define ledOff digitalWrite(onbd_led,LOW)
#define masterAddress 0x0001
#define masterChannel 0x17
#define maxIdleTime 1000          // 10 sec

class HX711 wieght(hx711_DT, hx711_PD_SCK);
as32ttl LORA; 

//int s[11];      //發射封包陣列
//int p[3];       //LoRa電量回覆參數命令陣列
//int P[2];       //暫存回覆電量封包陣列
//int a=0;        //按下STOP時為1
//int b=0;        //電量異常時為1
//int c=0;        //變化異常時為1

//int led1 = 10;    //STOP狀態LED紅燈號腳位
//int led2 = 11;    //Start狀態LED綠燈號腳位
//int led3 = 12;    //電量狀態LED黃燈號腳位

float calibration_factor = 60; //Load cell校正值

//long units;
//long units2=0;

//int stopButton =1;      //Stop按鈕狀態, normal pull-high, low active
//int startButton =1;     //Start按鈕狀態, normal pull-high, low active

//int WeightCount =0;   //計數測得重量值
//int EffectiveWeightCount=0;

union ftobytes{
  float f;
  byte b[4];
};

union ltobytes{
  unsigned long l;
  byte b[4];
};

union u16bytes{
  unsigned int u16;
  byte b[2];
};

void writeLoadScaleEEPROM(float s){
  ftobytes a;
  a.f=s;
  EEPROM[0]= a.b[0];
  EEPROM[1]= a.b[1];
  EEPROM[2]= a.b[2];
  EEPROM[3]= a.b[3];
}

float readLoadScaleEEPROM(){
  ftobytes a;
  a.b[0]= EEPROM[0];
  a.b[1]= EEPROM[1];
  a.b[2]= EEPROM[2];
  a.b[3]= EEPROM[3];
  return a.f;
}

void writeSumDeltaAlarmEEPROM(unsigned long a){
  ltobytes l;
  l.l = a;
  EEPROM[4]=l.b[0];
  EEPROM[5]=l.b[1];
  EEPROM[6]=l.b[2];
  EEPROM[7]=l.b[3];
}

unsigned long readSumDeltaAlarmEEPROM(){
  ltobytes l;
  l.b[0]=EEPROM[4];
  l.b[1]=EEPROM[5];
  l.b[2]=EEPROM[6];
  l.b[3]=EEPROM[7];
  return l.l;
}

void writeBatteryVoltageAlarmEEPROM(uint16_t a){
  u16bytes uxx;
  uxx.u16=a;
  EEPROM[8]=uxx.b[0];
  EEPROM[9]=uxx.b[1];
}

uint16_t readBatteryVoltageAlarmEEPROM(){
  u16bytes uxx;
  uxx.b[0]=EEPROM[8];
  uxx.b[1]=EEPROM[9];
  return uxx.u16;
}  

void flash3(){
  for(int i=0;i<3;i++){
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,HIGH);
      digitalWrite(blue_led,HIGH);
      delay(500);
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,LOW);
      digitalWrite(blue_led,LOW);
      delay(500);
  }
}

void flash1(){
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,HIGH);
      digitalWrite(blue_led,HIGH);
      delay(500);
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,LOW);
      digitalWrite(blue_led,LOW);
      delay(500);
}

void flashH(){
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,HIGH);
      digitalWrite(blue_led,HIGH);
}

void flashL(){
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,LOW);
      digitalWrite(blue_led,LOW);
}

void flash3a(){
  int i;
  for(i=0;i<3;i++){
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,LOW);
      digitalWrite(blue_led,HIGH);
      delay(100);
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,HIGH);
      digitalWrite(blue_led,LOW);
      delay(100);
  };
}

void flashroll(int8_t i,int dtime){
  uint8_t tog[3]={0,0,0};
  if (i<3) tog[i]=1;
  digitalWrite(red_led,tog[0]);
  digitalWrite(green_led,tog[1]);
  digitalWrite(blue_led,tog[2]);
  delay(dtime);
}

//#define p(x) Serial.println(x)

HardwareTimer &Timer = Timer1;
float wg[6];
float wgDelta[5];
float wgSumDelta;
float wgSumDeltaMax;
float wgSumDeltaMaxAlarm;

int cnt20=0;
int cnt1k=0;

int runState;
long idleTime;
uint8_t rcvBuffer[10];
int rcvlen;
float batteryV;
float batteryVMax;
int Bv;

void readHY711KeySample(){
  int i;
  if (runState==0){
    if (idleTime<maxIdleTime) idleTime++;
  }else idleTime=0;
  
  if (cnt20>=20){
    for(i=0;i<5;i++) wg[5-i]=wg[4-i];
    wg[0]=wieght.get_units();
    
    for(i=0;i<4;i++) wgDelta[4-i]=wgDelta[3-i];
    wgDelta[0]=abs(wg[1]-wg[0]);
    wgSumDelta = 0;
    for(i=0;i<5;i++) wgSumDelta=wgSumDelta+wgDelta[i];
    if (wgSumDelta > wgSumDeltaMax) wgSumDeltaMax=wgSumDelta;
    cnt20=0;
    Bv=analogRead(batteryVoltageIn);
    batteryV = ((float )Bv) * batteryScale;
  } else cnt20++;

  if (cnt1k>=90){
    if (cnt1k==90){
      if (runState){
        digitalWrite(red_led,LOW);
        digitalWrite(green_led,HIGH);
      } else {
        digitalWrite(red_led,HIGH);
        digitalWrite(green_led,LOW);
      }
      if (batteryV < batteryVMax) digitalWrite(blue_led,HIGH);
    }
    if (cnt1k>=100){
      digitalWrite(red_led,LOW);
      digitalWrite(green_led,LOW);
      digitalWrite(blue_led,LOW);
      cnt1k=0; 
    }else cnt1k++;
  } else cnt1k++; 
}


void setup() {  
  int startButton,stopButton;
  runState=0;

  analogReference(INTERNAL);          // 1.1V
  pinMode( start_bt ,INPUT_PULLUP);     //start switch
  pinMode( stop_bt ,INPUT_PULLUP);      //stop switch
   
  pinMode( red_led ,OUTPUT);     //STOP led
  pinMode( green_led ,OUTPUT);   //Start led
  pinMode( blue_led ,OUTPUT);    //Power led
  pinMode( onbd_led ,OUTPUT);    //On board led

  delay(10);
  startButton=digitalRead(start_bt);
  stopButton=digitalRead(stop_bt);

  int i,errorCode;
 
  LORA.begin(rf_md0, rf_md1, rf_aux); 
  
  if ((stopButton==LOW) && (startButton==LOW)){
    flashroll(0,100);
    flashroll(1,100);
    flashroll(2,100);
    flashroll(1,100);  
    LORA.setRecieverAddressChannel(0x0000,0x17);          // set LORA device address & channel for setting address
    int addr,chnl;
    addr = LORA.getRecieverAddress();
    chnl = LORA.getRecieverChannel();
    LORA.cleanSerialBuf();
    LORA.setMormalMode();                                 // set LORA to normal mode, and ready to reciever allocation device address and channel 
    
    uint8_t addrChnl[3],chyl[2];
    int rcvBytes=0;
    char str_buff[40];
    
    // recieve set address channel bytes stream
    // address(2 bytes) channel(1 bytes)
    
    for(i=0;i<rcvAddrChnlTime;i++){
      flashroll(0,300);
      flashroll(1,300);
      rcvBytes=LORA.available();
      if (rcvBytes>10) break;
      flashroll(2,300);
      flashroll(1,300);  
      sprintf(str_buff,"A:0x%04x C:0x%02x wait...\n",addr,chnl);
      LORA.send(masterAddress,masterChannel,str_buff);
    }
     
    if (rcvBytes>10){
      ledOn;
     
      addrChnl[0]=LORA.get();
      addrChnl[1]=LORA.get();
      addrChnl[2]=LORA.get();

      ltobytes wgsdm;
      wgsdm.b[0]=LORA.get();
      wgsdm.b[1]=LORA.get();
      wgsdm.b[2]=LORA.get();
      wgsdm.b[3]=LORA.get();

      u16bytes bvm;
      bvm.u16=0;
      bvm.b[0]=LORA.get();
      bvm.b[1]=LORA.get();

      chyl[0]=LORA.get();
      chyl[1]=LORA.get();

      if (wgsdm.l<10 || wgsdm.l>5000) writeSumDeltaAlarmEEPROM(50);
      else writeSumDeltaAlarmEEPROM(wgsdm.l);

      if (bvm.u16>440 || bvm.u16<290) writeBatteryVoltageAlarmEEPROM(330);
      else writeBatteryVoltageAlarmEEPROM(bvm.u16); 

      if (chyl[0]!='Y' || chyl[1]!='L'){
        LORA.send(masterAddress,masterChannel,"Error!!\n");  
        for(;;);
      }
      
      sprintf(str_buff,"Set A:0x%02x%02x C:0x%02x wD:%lu bl:%u \n",addrChnl[0],addrChnl[1], addrChnl[2],wgsdm.l,bvm.u16);
      LORA.send(masterAddress,masterChannel,str_buff);
      LORA.chkAUXH(1000);
      delay(1000);     
      LORA.setRecieverAddressChannel(addrChnl[0], addrChnl[1], addrChnl[2]);
      sprintf(str_buff,"Set A:0x%02x%02x C:0x%02x OK!\n",addrChnl[0],addrChnl[1], addrChnl[2]);
      LORA.send(masterAddress,masterChannel,str_buff);
      LORA.chkAUXH(1000);
      delay(1000);
      ledOff;
    }
    
    flashH();
    delay(2000);
    flashL();
    delay(500);
    softwareReset::simple();                              // finish set device address & channel
  }else{
    flashH();
    delay(300);
    flashL();
    delay(300);
    flashH();
    int j;
    for(j=0;j<50;j++)wieght.readf();   
    wieght.tare();
    delay(100);
    flashL();
    flash1();
    if ((stopButton==LOW) && (startButton==HIGH)){         // calibrate scale value
      float g200gV[3]={0.0 ,0.0 ,0.0};
      float stableV[2]={400.0,400.0};
      for(j=0;j<20;j++) flash3a();                         // flash LED, put 200g on hook
      do {
        g200gV[2]=g200gV[1];
        g200gV[1]=g200gV[0];
        g200gV[0]=wieght.get_value();
        stableV[0]=abs(g200gV[0]-g200gV[1]);
        stableV[1]=abs(g200gV[1]-g200gV[2]);
        flash3a();
      } while((g200gV[2]<700.0) || (g200gV[1]<700.0) || (g200gV[0]<700.0) || (stableV[0]>10.0) || (stableV[1]>10.0));
      calibration_factor = ((g200gV[0]+g200gV[1]+g200gV[2])/3.0)/200.0;
      wieght.set_scale(calibration_factor);  
      flash1();
      writeLoadScaleEEPROM(calibration_factor);             // store new scale value
      for(;;);
    } else {
      calibration_factor = readLoadScaleEEPROM();           //load old scale value
      wieght.set_scale(calibration_factor); 
    }
  }
  wg[0]=wieght.get_units();
  for(i=0;i<5;i++){
    wg[i+1]=wg[0];
    wgDelta[i]=0.0;
  }
  wgSumDelta=0.0;
  wgSumDeltaMax=0.0;
  wgSumDeltaMaxAlarm=(float )readSumDeltaAlarmEEPROM();
  if (wgSumDeltaMaxAlarm < 10.0 || wgSumDeltaMaxAlarm > 5000.0){
    wgSumDeltaMaxAlarm=50.0;
    writeSumDeltaAlarmEEPROM(50);   
  }
  
  Bv = analogRead(batteryVoltageIn);
  batteryV = ((float )Bv) * batteryScale;
  float bvmax = readBatteryVoltageAlarmEEPROM()/100.0;
  if (bvmax > 4.4 || bvmax < 2.9) {
    writeBatteryVoltageAlarmEEPROM(330);
    batteryVMax=3.3;     
  }else batteryVMax=bvmax;
  LORA.setMormalMode();
  Timer.init(10000);    // 10ms interval
  Timer.attachInterrupt(readHY711KeySample); 
}

void wakeUp(){
  sleep_disable();//Disable sleep mode
  detachPinChangeInterrupt(start_bt); //Removes the interrupt from pin 2;
}

void loop() {     

  if (digitalRead(start_bt)==0) runState=1;
  if (digitalRead(stop_bt)==0) runState=0;

  if (idleTime>=maxIdleTime){     // go to sleep
    Timer.detachInterrupt();
    digitalWrite(red_led,LOW);    // turn off all of led
    digitalWrite(green_led,LOW);
    digitalWrite(blue_led,LOW);
    LORA.setSleepMode();
    wieght.power_down();
    attachPinChangeInterrupt(start_bt, wakeUp, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_SAVE); 
    sleep_enable();
    sleep_cpu();
    sleep_disable(); 
    wieght.power_up();
    LORA.setMormalMode();  
    Timer.attachInterrupt(readHY711KeySample); 
    runState=1;
  }

  delay(100);
  int rcvBytes;
  rcvBytes = LORA.available();
  
  if (rcvBytes > 0) {
    LORA.gets(rcvBuffer);           // read master knock message {addressHigh,addressLow,0xff,0x55,0x00} 
    LORA.cleanSerialBuf();
    if (rcvBuffer[0]==0x00 && rcvBuffer[1]==0x01 && rcvBuffer[2]==0x17 && rcvBuffer[3]==0xff && rcvBuffer[4]==0x55){    // check data format
      char txBuffer[14];
      txBuffer[0]=0x55;           // check code
      txBuffer[1]=LORA.getRecieverAddressH();
      txBuffer[2]=LORA.getRecieverAddressL();
      txBuffer[3]=LORA.getRecieverChannel();
      noInterrupts();
      if (runState) {
        txBuffer[4] =(uint8_t) (((long)wg[0]) & 0xff);
        txBuffer[5] =(uint8_t) ((((long)wg[0]) & 0xff00)>>8);
        txBuffer[6] =(uint8_t) ((((long)wg[0]) & 0xff0000)>>16);
        txBuffer[7]=(uint8_t) ((((long)wg[0]) & 0xff000000)>>24);
      } else {
        txBuffer[4] = 0;
        txBuffer[5] = 0;
        txBuffer[6] = 0;
        txBuffer[7] = 0;
      }
      txBuffer[8]=(uint8_t) (((unsigned long)wgSumDeltaMax) & 0xff);
      txBuffer[9]=(uint8_t) ((((unsigned long)wgSumDeltaMax) & 0xff00)>>8);
      txBuffer[10]=(uint8_t) ((((unsigned long)wgSumDeltaMax) & 0xff0000)>>16);
      txBuffer[11]=(uint8_t) ((((unsigned long)wgSumDeltaMax) & 0xff000000)>>24);
      txBuffer[12]=0x00 | ((runState)? 0x02:0x00) | ((batteryV < batteryVMax)? 0x01:0x00) | ((wgSumDeltaMax>wgSumDeltaMaxAlarm)? 0x04:0x00);
      wgSumDeltaMax=0.0; 
      interrupts();
      txBuffer[13]=0x43; 
      
      LORA.send(masterAddress,masterChannel,txBuffer,sizeof(txBuffer));    
    }
  } 
}           
 
