#include "HX711.h"
HX711 scale(4, 5); //DT 4, CLK 5
int s[11];        //發射封包陣列
int p[3];         //LoRa電量回覆參數命令陣列
int P[2];         //暫存回覆電量封包陣列
int a=0;       //按下STOP時為1
int b=0;       //電量異常時為1
int c=0;       //變化異常時為1
int led1 = 10;    //STOP狀態LED紅燈號腳位
int led2 = 11;    //Start狀態LED綠燈號腳位
int led3 = 12;    //電量狀態LED黃燈號腳位
//float calibration_factor = 1020;
float calibration_factor = 60; //Load cell校正值
long units;
long units2=0;
int Stopbutton =0;       //Stop按鈕狀態
int initialbutton =0;    //initialbutton按鈕狀態
int WeightCount =0;   //計數測得重量值
void setup() {
   pinMode( 7 ,OUTPUT);       //MD0,MD1的狀態
   pinMode( 8 ,INPUT);        //STOP
   pinMode( 9 ,INPUT);        //initial
   pinMode( led1 ,OUTPUT);    //STOP
   pinMode( led2 ,OUTPUT);   //Start
   pinMode( led3 ,OUTPUT);  //電量
   Serial.begin(9600);      //設定 baudrate
   scale.set_scale(calibration_factor);  
   scale.tare(); 
}
void loop() {        
     Stopbutton = digitalRead(8);        //Stop按鈕腳位
     initialbutton = digitalRead(9);     //Start按鈕腳位
  units = scale.get_units(),10;      //讀取load cell data  
//一般存放點滴重量陣列 
           //陣列s[0]~s[1]為目標位址(RX)
           s[0]={0x00};
           s[1]={0x01};         
           s[2]={0x01};
           //陣列s[3]~s[5]為自身位址(TX)
           s[3]={0x00};
           s[4]={0x03}; 
           s[5]={0x01};
          //陣列s[6]~s[9]為點滴重量
           s[6]={(units >> 24) & (0xFF)}; 
           s[7]={(units >> 16) & (0xFF)};
           s[8]={(units >> 8)  & (0xFF)};
           s[9]={(units) & (0xFF) };   
    if (Serial.read() == (0x00,0x01,0x01,0x11,0xFF,0x55,0x00)) 
      {    
        for(int a=0;a<11;a++)  //傳輸點滴重量封包至LoRa模組發射端
           {   
            Serial.write(s[a]); 
           }
         Serial.end(); 
         Serial.begin(9600); 
       }                         
       
//Stop狀態  
    if(Stopbutton == HIGH)  
       {
         while (!Serial.available()) {}  //等待模塊回覆之封包  
         a=1;
         do{
           units=0;     //將點滴重量值設為0
           //陣列s[0]~s[1]為目標位址(RX) 
           s[0]={0x00};
           s[1]={0x01};         
           s[2]={0x01};
           //陣列s[3]~s[5]為自身位址(TX)
           s[3]={0x00};
           s[4]={0x03}; 
           s[5]={0x01};
          //陣列s[6]~s[9]為點滴重量
           s[6]={(units >> 24) & (0xFF)}; 
           s[7]={(units >> 16) & (0xFF)};
           s[8]={(units >> 8)  & (0xFF)};
           s[9]={(units) & (0xFF) };            
 if(Serial.read() == (0x00,0x01,0x01,0x55,0x43)) 
    {    
     for(int a=0;a<11;a++) //發送陣列內封包
        {   
          Serial.write(s[a]); 
           }
          Serial.end(); 
          Serial.begin(9600); 
             }         
          digitalWrite(led1, HIGH); 
                }while(!initialbutton == HIGH && !Stopbutton == HIGH); 
                    }
//初解除STOP狀態
 if(initialbutton == HIGH )    //initial button，初始化Start狀態
   {       
     while (!Serial.available()) {}  //等待輪詢封包       
     if(Serial.read() == (0x00,0x01,0x01,0x55,0x43)) 
       {    
         for(int a=0;a<11;a++) //發送陣列內封包
            {   
            Serial.write(s[a]); 
             }
            Serial.end(); 
            Serial.begin(9600); 
        }        digitalWrite(led1, LOW); 
                 digitalWrite(led2, HIGH);
                 delay(500);
                 digitalWrite(led2, LOW);  
          }
//電量偵測
if( EffectiveWeightCount > 1000)
  {
    digitalWrite(7,1); // MD0,MD1設為高電位
    delay(25);
    p[0]={0xC5};  //詢問模塊電量之參數命令
    p[1]={0xC5};
    p[2]={0xC5};
    for(int a=0;a<3;a++)//詢問封包發送
        {   
        Serial.write(p[a]); 
          }       
while (!Serial.available()) {}//等待模組回覆電量封包
        for(int a=0;a<2;a++)
            {   
            P[a]= {Serial.read()}; //存取收到模組電量
              }
            if( P[a] < (P[a]*0.5) ) 
              {    
              digitalWrite(led3,1);   //電量異常燈號亮起         
              digitalWrite(7,0);   // MD0,MD1設為低電位
              b=1; 
              EffectiveWeightCount = 0;  
                }else{
                    digitalWrite(led3,0);   //電量異常燈號滅掉
                    digitalWrite(7,0);     // MD0,MD1設為低電位
                    b=0; 
                    EffectiveWeightCount = 0; 
                    }
                     digitalWrite(7,0);   // MD0,MD1設為低電位 
                     EffectiveWeightCount = 0;           
                    }else{
                       digitalWrite(7,0);   // MD0,MD1設為低電位
                          }     
                            }                                            
