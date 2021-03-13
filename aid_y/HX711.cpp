/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include <Arduino.h>
#include "HX711.h"


HX711::HX711() {
    int i;
    for(i=0;i<12;i++){
      vbuff[i]=0.0;
      lbuff[i]=0;
    }
}

HX711::HX711(byte dout, byte pd_sck, byte gain = 128) {
  begin(dout,pd_sck,gain);    
}

HX711::~HX711() {
}

void HX711::begin(byte dout, byte pd_sck, byte gain) {
	PD_SCK = pd_sck;
	DOUT = dout;

	pinMode(PD_SCK, OUTPUT);
	pinMode(DOUT, INPUT);

  gainX=gain;
	set_gain(gain);
}

bool HX711::is_ready() {
	return digitalRead(DOUT) == LOW;
}

void HX711::set_gain(byte gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

	digitalWrite(PD_SCK, LOW);				// Power Up
	readf();													// read first
}

int long_cmp(const void *a, const void *b) 
{ 
    const long *ia = (const long *)a; // casting pointer types 
    const long *ib = (const long *)b;
    return (*ia > *ib)? -1 : ((*ia < *ib)? 1 : 0);
} 


long HX711::readf() {

  // Wait for the chip to become ready.
  wait_ready(1);

  // Define structures for reading data into.
  //unsigned long value = 0;
  
  // Disable interrupts.
  noInterrupts();
  
  long adc_data;
  adc_data=0;
  int data_pin;
  int i;

  for(i=0;i<24;i++){
    digitalWrite(PD_SCK, HIGH);
    delayMicroseconds(1);   
    digitalWrite(PD_SCK, LOW);
    data_pin=digitalRead(DOUT);
    adc_data=(adc_data<<1)|data_pin;
    delayMicroseconds(1);   
  }

  adc_data = (adc_data<<8)/256;
  
  // Set the channel and the gain factor for the next reading using the clock pin.
  for (i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH); 
    delayMicroseconds(1); 
    digitalWrite(PD_SCK, LOW);
    delayMicroseconds(1);
  }

  // Enable interrupts again.
  interrupts();

  for(i=0;i<11;i++) lbuff[11-i]=lbuff[10-i];
  lbuff[0]=adc_data;
  long tbuff[12];
  for(i=0;i<12;i++) tbuff[i]=lbuff[i];

  qsort(tbuff, 12, sizeof(tbuff[0]), &long_cmp);

  float sum=0.0;
  for(i=3;i<9;i++) sum=sum+((float) tbuff[i]);
  sum=sum+tbuff[1]*0.4;
  sum=sum+tbuff[10]*0.4;
  sum=sum+tbuff[2]*0.2;
  sum=sum+tbuff[9]*0.2;
  
  return ((long)(sum/7.2));
}


void HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	}
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

float HX711::read_average(int times) {
	float sum = 0.0;
	for (int i = 0; i < times; i++) {
		sum += (float) readf();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(20);
	}
	return sum / ((float) times);
}

float HX711::get_value(int times) {
	return read_average(times) - OFFSET;
}

float HX711::get_value(){
  return ((float) readf()) - OFFSET;
}

float HX711::get_units(int times) {
	return get_value(times) / SCALE;
}

float HX711::get_units() {
 return get_value() / SCALE;
}
void HX711::tare(int times) {
	float sum = read_average(times);
	set_offset(sum);
}

int float_cmp(const void *a, const void *b) 
{ 
    const float *ia = (const float *)a; // casting pointer types 
    const float *ib = (const float *)b;
    return (*ia > *ib)? -1 : ((*ia < *ib)? 1 : 0);
} 

void HX711::tare() {
  int i;
  for(i=0;i<12;i++){
    vbuff[i]=(float) readf();
    delay(50);
  };
  
  qsort(vbuff, 12, sizeof(vbuff[0]), &float_cmp);

  float sum=0;
  
  for(i=1;i<11;i++) sum=sum+vbuff[i];
  
  set_offset(sum/10.0);
}

void HX711::set_scale(float scale) {
	SCALE = scale;
}

float HX711::get_scale() {
	return SCALE;
}

void HX711::set_offset(float offset) {
	OFFSET = offset;
}

float HX711::get_offset() {
	return OFFSET;
}

void HX711::power_down() {
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

void HX711::power_up() {
	set_gain(gainX);
}
