//=======================================================================
// SMD Soldering Station with AVR
// (c) 2016 Manos Mar <manolena2001@gmail.com>, https://manolena.wordpress.com/2015/12/28/diy-smd-soldering-station-with-avr/
// (c) 2014 Martin Kumm http://www.martin-kumm.de/wiki/doku.php?id=Projects:SMD_Solderstation/
// (c) 2014 MatthiasW  https://debugginglab.wordpress.com/2014/10/30/soldering-station/
//=======================================================================
//    LIBRARIES
//=======================================================================
#include <Adafruit_GFX.h>    
#include <Adafruit_TFTLCD.h> 
#include <SPI.h>
#include <SD.h>
#include <EEPROMEx.h>
#include <Encoder1.h>
#include <PinChangeInt.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include "DEFINES.h"
//=======================================================================
//    INSTANCES
//=======================================================================
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//=======================================================================
//    SETUP
//=======================================================================
void setup(void) 
{ 
  Serial.begin(9600); 
  pinMode(PWMpin, OUTPUT);
  setPwmFrequency(PWMpin, PWM_DIV);
  digitalWrite(PWMpin, HIGH); 
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) 
  {
    readings[thisReading] = 0;
  }  
  
  tft.reset();
  tft.begin(MCUFRIEND_2_4_TFT);
  tft.setRotation(ANGLE_270);//
  Serial.println(F("TFT Reset."));
  delay(10);

  pinMode(STANDBYin, INPUT_PULLUP);

  pinMode(TEMPin, INPUT);
  digitalWrite(TEMPin, LOW);  

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 
  beepBuzzer(6250,250);  

  pinMode(HEAT_LED, OUTPUT);
  digitalWrite(HEAT_LED, LOW);   

  pinMode(Encoder1ChnA, INPUT); digitalWrite(Encoder1ChnA, HIGH); //turn pullup resistor on
  pinMode(Encoder1ChnB, INPUT); digitalWrite(Encoder1ChnB, HIGH); //turn pullup resistor on
  pinMode(EncoderDetent, INPUT); digitalWrite(EncoderDetent, HIGH); //turn pullup resistor on 
  
  PCintPort::attachInterrupt(Encoder1ChnA, &updateEncoder_ISR, CHANGE); 
  PCintPort::attachInterrupt(Encoder1ChnB, &updateEncoder_ISR, CHANGE);
  PCintPort::attachInterrupt(EncoderDetent, &EncoderClick_ISR, FALLING);
  myEncoder.setRate(1.0f);
  myEncoder.setMinMax(MIN_TEMP,MAX_TEMP); 

  Timer1.initialize(150000); // set a timer of length 150000 microseconds (or 0.15 sec)
  Timer1.attachInterrupt( timer1_ISR ); // attach the service routine here  

  Serial.println(F("Initializing SD card..."));
#if defined (__AVR_ATmega2560__) || (__AVR_ATmega1280__)
  if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_SCK )) 
  {
    Serial.println(F("Initialization failed!"));
    return;
  }
#else
  if (!SD.begin(SD_CS)) 
  {
    Serial.println(F("Initialization failed!"));
    return;
  }
#endif

  Serial.println("Sketch'es location:");
  Serial.println("C:\\Users\\Administrator\\Documents\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\IRON_18");

  Serial.println(F("Initialization done."));
  delay(500);

  SPI.setClockDivider(SPI_CLOCK_DIV4);  // 4MHz 

  tft.fillScreen(tft.color565(255,255,255));//BLACK  
  bmpDraw("FILE0.bmp", 0, 0);
  delay(2000);
  
  mainScreen();  

  will_temp = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  myEncoder.setPosition(will_temp);
  
  if (will_temp == 4294967295) 
  {
    will_temp = STANDBY_TEMP;
  }
  encoderPos = EEPROM.readInt(ENCODER_EEPROM_ADDRESS);
  if (encoderPos == 4294967295) 
  {    
    myEncoder.setPosition(will_temp);
  }

#if defined BARGRAPHS  
  verticalBar();
  horizontalBar();
#endif 
  Input = getTemperature();
  Setpoint = will_temp;
  myPID.SetMode(AUTOMATIC);
  pwm = 0;  
}
//=======================================================================
//    MAIN LOOP
//=======================================================================
void loop()
{  
  unsigned long now = millis();

  if(standby_act == false)
  {
    Input = getTemperature();
    Setpoint = encoderPos;
    encoderPosTemp = encoderPos;
  }
  else
  {
    Input = getTemperature();
    Setpoint = STANDBY_TEMP;
  }    
  double gap = abs(Setpoint-Input); 
  if(gap < 10)
  {  
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();  
  
  static boolean oneTime = false; 

  will_temp = map(encoderPos, MIN_TEMP, MAX_TEMP, MIN_TEMP, MAX_TEMP);

  int tempWill = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  tft.setTextSize(3);  

  if (digitalRead(STANDBYin) == HIGH)
  {
    tft.setCursor(2,2);
    if(oneTime)
    {      
      tft.fillRect(2, 2, 80, 40, tft.color565(255,255,255));
      oneTime = false; 
    }     
    tft.setTextColor(tft.color565(255,0,255));//GREEN
    tft.print("OPP ");
  }  
  else
  {
    tft.setCursor(2,2);
    if(!oneTime)
    {
      tft.fillRect(2, 2, 80, 40, tft.color565(255,255,255));
      oneTime = true;
    }
    tft.setTextColor(tft.color565(0,255,255));
    tft.print("STBY");
  }

  if((memNoWrite == true) && (memWrite = false))
  {    
    tft.setTextColor(tft.color565(255,255,255));    
    memNoWrite = true; 
    memWrite = false; 
  }  
  else if((memNoWrite == false) && (memWrite = true))
  {
    tft.fillRect(90, 2, 170, 20, tft.color565(255,255,255));
    tft.setTextColor(tft.color565(255,0,255));  //GREEN  
    tft.setTextSize(2);
    tft.setCursor(100,7);
    tft.print("*MEM = ");   
    tft.print(tempWill);
    tft.setTextSize(1);
    tft.print(" o"); 
    tft.setTextSize(2);
    tft.print("C    ");
  }
 
  memNoWrite = true; 
  memWrite = false; 

  int will_temp_tmp = will_temp;
  
  actual_temperature = getTemperature();
  if((actual_temperature >= MAX_TEMP) && (actual_temperature < MAX_TEMP+100))
  {    
    pwm = 0;
    digitalWrite(HEAT_LED, LOW);
    tft.fillScreen(tft.color565(255,255,255));//BLACK
    do
    {
      tft.setTextColor(tft.color565(0,255,255));
      tft.setTextSize(5);
      tft.setCursor(5,80);
      tft.print("UNPLUGGED!");
      actual_temperature = getTemperature();
    }
    while(actual_temperature >= MAX_TEMP);
   
    mainScreen();
    
    tft.setTextSize(7);
    tft.setCursor(100,108);
    tft.setTextColor(tft.color565(255,0,255));
    if (t2 < 100)
      tft.print(" ");
    if (t2 <10)
      tft.print(" ");
    tft.print(t2);
  }

  pwm = Output;
  MAX_PWM = actual_temperature <= STANDBY_TEMP ? MAX_PWM_LOW : MAX_PWM_HI;
  pwm = pwm > MAX_PWM ? pwm = MAX_PWM : pwm < 0 ? pwm = 0 : pwm;
  analogWrite(PWMpin, pwm);

  if(pwm != 0)
  {
    heater = HIGH;
    tft.fillCircle(150, 190, 11, tft.color565(0,255,255));
  }
  else 
  {    
    heater = LOW;
    tft.fillCircle(150, 190, 11, tft.color565(255,255,255));
  } 

  if (standby_act && (will_temp >= STANDBY_TEMP ))
  {
    will_temp = STANDBY_TEMP;  
  }  
  writeHEATING(will_temp, Input, pwm);
}
//=======================================================================
//    END OF PROGRAM
//=======================================================================

