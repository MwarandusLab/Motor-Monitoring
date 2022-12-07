#include <SoftwareSerial.h>
#include <Wire.h>  //lcd I2C library
#include <Adafruit_INA219.h> //current and voltage sensor library
#include <LiquidCrystal_I2C.h> //lcd I2C display adapter library

//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(4, 5); //SIM800L Tx & Rx is connected to Arduino #3 & #2
//Create software serial object to communicate with LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

char incomingByte;
String inputString;

int Relay = 3;
int smsSent=0;
int smsSent1 =0;
int smsSent2 =0;
int smsSent3 =0;
int Buzzer = 2;
int Lm35 = A0;
int Red_Led = 6;
int Green_Led = 9;
int vibrationSensor = A1;
int vibrationValue;

Adafruit_INA219 ina219;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  
  pinMode(Red_Led, OUTPUT);
  pinMode(Green_Led, OUTPUT);
  
  //lcd.begin();
  //lcd.backlight();
  
  pinMode(Relay, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Red_Led, HIGH);
  
  
  while(!mySerial.available()){
    mySerial.println("AT");
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected!");
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  delay(500);
  mySerial.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  delay(500);
  mySerial.println("AT+CMGL=\"REC UNREAD\""); //Read Unread Messages
  
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
  
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_1A();
  
  /*lcd.clear();
  lcd.setCursor(5,0);
  lcd.print("SYSTEM");
  lcd.setCursor(2,1);
  lcd.print("INITIALIZING");
  delay(1000);*/
}

void loop() {
  updateSerial();
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.println("");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  delay(1000);
  
  int sensorValue = analogRead(Lm35);
  float voltage1 = sensorValue * (5.0 / 1023.0);
  float temperature = voltage1*100;
  Serial.print("Temperature: ");
  Serial.print(temperature,1);
  Serial.println(" Celcius");  
  Serial.println("");

  if(temperature > 40.0){
    smsSent = 0;
    Serial.println("");
    Serial.print("ALERT! HIGH TEMPERATURE DETECTED");
    Serial.println("");
    digitalWrite(Buzzer, HIGH);
    digitalWrite(Relay,LOW);
    digitalWrite(Green_Led, LOW);
    if(smsSent3 == 0){
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      mySerial.print("ALERT! HIGH TEMPERATURE DETECTED"); //text content
      updateSerial();
      mySerial.write(26);
      delay(1000);
      smsSent3 = 1;
      delay(10000);
      mySerial.println("ATD+ +254702481917;");
      updateSerial();
      delay(2000);
    }
    
  }else if(temperature < 40.0 ){
    Serial.println("");
    digitalWrite(Buzzer, LOW);
    smsSent = 0;
  }     
  
  vibrationValue = analogRead(vibrationSensor);
  
  if(mySerial.available()){  
    delay(100);
    while(mySerial.available()){
      incomingByte = mySerial.read();
      inputString += incomingByte;
   }
    delay(1000);
    
    Serial.println(inputString);
    inputString.toUpperCase(); //uppercase the received Message
  
  if(loadvoltage < 1.7 && current_mA > 20.0){
    smsSent = 0;
    Serial.println("");
    Serial.print("ALERT!! UNDER VOLTAGE DETECTED AND MOTOR IS ON ");
    Serial.println("");
    digitalWrite(Buzzer, HIGH);
    if(smsSent == 0){
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      mySerial.print("ALERT!! UNDER VOLTAGE DETECTED AND MOTOR IS ON "); //text content
      updateSerial();
      mySerial.write(26);
      smsSent = 1;
      delay(10000);
      mySerial.println("ATD+ +254702481917;");
      updateSerial();
      delay(2000);
      
    }
      
  }else {
    Serial.println("");
    Serial.print("ALERT!! UNDER VOLTAGE DETECTED AND MOTOR IS OFF");
    Serial.println("");
    delay(1000);
    smsSent = 0;
  }
      
  if((loadvoltage > 2.5 && loadvoltage < 3.8) && current_mA > 20.0 ){
    if(vibrationValue < 1023){
      Serial.println("");
      Serial.print("VIBRATION: DETECTED");
      Serial.println("");
      digitalWrite(Buzzer, HIGH);
      digitalWrite(Relay,LOW);
      digitalWrite(Green_Led, LOW);
      if(smsSent1 == 0){
        mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
        updateSerial();
        mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
        updateSerial();
        mySerial.print("VIBRATION: DETECTED"); //text content
        updateSerial();
        mySerial.write(26);
        delay(10);
        smsSent1=1;
        delay(10000);
        mySerial.println("ATD+ +254702481917;");
        updateSerial();
        delay(2000);
      }
      
    }else if(vibrationValue >= 1023){
      smsSent=0;
      Serial.println("");
      Serial.print("VIBRATION: NOT DETECTED");
      Serial.println("");
      digitalWrite(Buzzer, LOW);
      
    }
  }
    delay(10);
  if(loadvoltage > 3.0 && current_mA > 20.0){
    smsSent2 = 0;
    Serial.println("");
    Serial.print("ALERT!! OVER VOLTAGE DETECTED AND MOTOR IS ON ");
    Serial.println("");
    digitalWrite(Buzzer, HIGH);
    digitalWrite(Relay,LOW);
    if(smsSent2 == 0){
      mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
      updateSerial();
      mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
      updateSerial();
      mySerial.print("ALERT!! OVER VOLTAGE DETECTED AND MOTOR IS ON"); //text content
      updateSerial();
      mySerial.write(26);
      delay(1000);
      smsSent2 = 1;
      
      delay(10000);
      mySerial.println("ATD+ +254702481917;");
      updateSerial();
      delay(2000);

      
    }
      
  }else{
    Serial.println("");
    Serial.print("ALERT!! OVER VOLTAGE DETECTED AND MOTOR IS OFF");
    Serial.println("");
    digitalWrite(Relay,LOW);
    digitalWrite(Green_Led, LOW);
    smsSent=0;
  }
    delay(1000);
  if(Relay == HIGH){
    digitalWrite(Green_Led, HIGH);    
  }else {
    digitalWrite(Green_Led, LOW);
  }
  //turn RELAY ON OR OFF
  if(inputString.indexOf("ON") > -1){
    digitalWrite(Relay, HIGH);
    digitalWrite(Green_Led, HIGH);
    mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    updateSerial();
    mySerial.print("MOTOR IS ON"); //text content
    updateSerial();
    mySerial.write(26);
    delay(1000);
  }
  if(inputString.indexOf("OFF") > -1){
    digitalWrite(Relay, LOW);
    mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    updateSerial();
    mySerial.print("MOTOR IS OFF"); //text content
    updateSerial();
    mySerial.write(26);
    delay(1000);
  }
  if(inputString.indexOf("STATUS") > -1){
    mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    mySerial.println("AT+CMGS=\"+254702481917\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    updateSerial();
    mySerial.println("      MOTOR STATUS  "); //text content
    mySerial.print("Load Voltage: "); //text content
    mySerial.print(loadvoltage);
    mySerial.println(" V");
    mySerial.print("Current: "); //text content
    mySerial.print(current_mA);
    mySerial.println(" mA");
    mySerial.print("Power: "); //text content
    mySerial.print(power_mW);
    mySerial.println(" mW");
    mySerial.print("Temperature: "); //text content
    mySerial.print(temperature,1);
    mySerial.println(" C");
    updateSerial();
    mySerial.write(26);
    delay(1000);
  }

    //delete Messages & Save Memory
    
  if(inputString.indexOf("OK") == -1){
    mySerial.println("AT+CMGDA=\"DEL ALL\"");

    delay(1000);
    }
    inputString = "";
  }

}

void updateSerial(){
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
