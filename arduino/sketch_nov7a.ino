
#include <Wire.h>
#include <VL53L0X.h>
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

VL53L0X sensor;
int flag=0;
int counter=0;

void setup(){
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(3);
  tft.setTextColor (TFT_WHITE);
  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(200);
  sensor.startContinuous();

}

void loop(){
  int distance =sensor.readRangeContinuousMillimeters();
  if(flag ==1){
    tft.fillScreen(TFT_GREEN);
    char distancechar[16];
    itoa(distance,distancechar,10);
    if (distance<150){
      tft.fillScreen(TFT_RED);
      tft.drawString(distancechar,30,30,8);
      counter++;
      if (counter>5){
        Serial.print(1);
        counter=0;
        flag=0;
      }
    }
    if (150<distance && distance<500){
      tft.drawString(distancechar,30,30,8);
      counter=0;
    }
  }
  else{
    tft.fillScreen(TFT_BLACK);
    if (Serial.available()) {
      if (Serial.read()){
        flag=1;
      }
    }
  }
  delay(300);
}