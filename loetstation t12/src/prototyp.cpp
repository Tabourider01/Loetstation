#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_MCP9600.h"

#define OUT 3

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;

#define DT_SW 12
#define CLK_SW 11
#define SW 13

Adafruit_MCP9600 mcp;
#define mcpADD 0x67
float istTemp;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int counter = 0;
int lastCounter;
int stateRotary;
int lastStateRotary;

void setupDisplay(){
  lcd.init();
  lcd.backlight();

  lcd.setCursor(4, 0);
  lcd.print("Soll:000");
  lcd.setCursor(4, 1);
  lcd.print("Ist :");
}
//* Diese Funktion setzt den MCP9600 temperatusconverter auf
void setupMCP(){
  if (! mcp.begin(mcpADD)){
    Serial.println("MCP not found");
    while (1);
  }
  Serial.println("Found MCP9600!");
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_N);
  Serial.print("Thermocouple type set to ");
  switch (mcp.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp.setFilterCoefficient(3);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp.getFilterCoefficient());

  mcp.setAlertTemperature(1, 30);
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp.getAlertTemperature(1));
  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);

  Serial.println(F("------------------------------"));
}

void setup() {
  Serial.begin(9600);

  Input = istTemp;
  Setpoint = counter;

  pinMode(DT_SW, INPUT_PULLUP);
  pinMode(CLK_SW, INPUT_PULLUP);
  pinMode(SW, INPUT);

  setupDisplay();
  setupMCP();
  lastStateRotary = digitalRead(DT_SW);
  Serial.println("---Setup Done---");
}


//*Funktion funktioniert
void rotary(){

  stateRotary = digitalRead(DT_SW);
  if((stateRotary != lastStateRotary) && (stateRotary == 0)){
    if(digitalRead(CLK_SW) != stateRotary){
      counter--;
    }
    else{
      counter++;
    }
  }
  lastStateRotary = stateRotary;
}
//* Auslesen und schreiben auf das display geht
void displaySollTemp(){
  if (counter != lastCounter){
    lcd.setCursor(9, 0);  
    if (counter < 10){
      lcd.print("00");
      lcd.print(counter);
    }
    else if (counter < 100){
      lcd.print("0");
      lcd.print(counter);
    }
    else{
      lcd.print(counter);
    }
    lastCounter = counter;

    Serial.println(counter);
  }
}
//todo Funktion lesen von Temp
int readIstTemp(){

  mcp.readThermocouple();
  //Serial.print("Hot Junction: "); Serial.println(mcp.readThermocouple());
  //Serial.print("ADC: "); Serial.print(mcp.readADC() * 2); Serial.println(" uV");
  istTemp = mcp.readThermocouple();
  delay(1000);
  return istTemp;
}
//todo Funktion anzeigen temp 
void displayIstTemp(){

}
//todo PID loop mit PWM für ansteuerung von mosfet, der dann heizt
void heizen(){

  Input = readIstTemp();;
  analogWrite(OUT, Output);

}

void loop() {

  rotary();
  displaySollTemp();
  heizen();

}

