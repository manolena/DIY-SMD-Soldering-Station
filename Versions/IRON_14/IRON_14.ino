#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>
#include <EEPROMEx.h>
#include <Encoder1.h>
#include <PinChangeInt.h>
#include <TimerOne.h>
#include <PID_v1.h>
//=======================================================================
#define MCUFRIENT_2_4_TFT  0x9341

#if defined (__AVR_ATmega2560__) || (__AVR_ATmega1280__)
#define SD_CS                 10     // Set the chip select line to whatever you use (10 doesnt conflict with the library)
#define SD_SCK                13
#define SD_MISO               12
#define SD_MOSI               11
#endif

#define LCD_CS                A3 // Chip Select goes to Analog 3
#define LCD_CD                A2 // Command/Data goes to Analog 2
#define LCD_WR                A1 // LCD Write goes to Analog 1
#define LCD_RD                A0 // LCD Read goes to Analog 0
#define LCD_RESET             A4 // Can alternately just connect to Arduino's reset pin

#define STANDBYin             47
#define TEMPin 	              A15
#define PWMpin 	              16

#define Encoder1ChnA          78
#define Encoder1ChnB          77
#define EncoderDetent         79

#define BUZZER_PIN            48
#define HEAT_LED              49

// 16-bit color values:
#define BLACK           0x0000      /*   0,   0,   0 */
#define NAVY            0xFFF0      /*   0,   0, 128 */
#define DARKGREEN       0xFC1F      /*   0, 128,   0 */
#define DARKCYAN        0xFC10      /*   0, 128, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY       0xC618      /* 192, 192, 192 */
#define DARKGREY        0x7BEF      /* 128, 128, 128 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0x07FF      /* 255, 255,   0 */
#define WHITE           0xFFFF      /* 255, 255, 255 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */
#define PINK            0xF81F

#define ANGLE_0    0
#define ANGLE_90   1
#define ANGLE_180  2
#define ANGLE_270  3

#define VERSION "v1.5"		
#define INTRO

#define DELAY_MAIN_LOOP 	1//150
#define DELAY_MEASURE 		2

#define ADC_TO_TEMP_GAIN 	0.99//2.50//0.53 //0.415
#define ADC_TO_TEMP_OFFSET      25.0
#define STANDBY_TEMP		175
#define MAX_TEMP		400

#define MAX_PWM_LOW		180//180
#define MAX_PWM_HI		210//210		

#define PWM_DIV                 1024		

#define WILL_TEMP_EEPROM_ADDRESS 0x10
#define ENCODER_EEPROM_ADDRESS   0x20

//#define DEBUG_SER
#define BARGRAPHS

double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
//double aggKp=3.6, aggKi=2, aggKd=3.6;
//double consKp=0.9, consKi=0.5, consKd=0.9;


double aggKp=320, aggKi=169, aggKd=31.5;
double consKp=320, consKi=169, consKd=31.5;

//=======================================================================
int pwm = 0; //pwm Out Val 0.. 255
int actual_temperature, will_temp = STANDBY_TEMP;

boolean standby_act = false;
int tempDIV;
float encoderValue = 0;
volatile float encoderPos = 0;
volatile float encoderPosTemp = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;
boolean memWrite = false;
boolean memNoWrite = true;
boolean state = false;
boolean heater = false;
boolean unplug = 0;
int t1,t2;
static boolean rotating = false;
const int numReadings = 30;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
float adcValue;

//=======================================================================
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//=======================================================================
void setup(void) 
{ 
  Serial.begin(9600); 
  for (int thisReading = 0; thisReading < numReadings; thisReading++) 
  {
    readings[thisReading] = 0;
  }  
  tft.reset();
  tft.begin(MCUFRIENT_2_4_TFT);
  tft.setRotation(ANGLE_270);//
  Serial.println(F("TFT Reset."));
  delay(10);

  pinMode(STANDBYin, INPUT_PULLUP);

  pinMode(PWMpin, OUTPUT);
  digitalWrite(PWMpin, LOW);

  pinMode(TEMPin, INPUT);
  digitalWrite(TEMPin, LOW);  

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 
  beepBuzzer(6250,250);

  setPwmFrequency(PWMpin, PWM_DIV);
  digitalWrite(PWMpin, LOW); 

  pinMode(HEAT_LED, OUTPUT);
  digitalWrite(HEAT_LED, LOW);   

  pinMode(Encoder1ChnA, INPUT); digitalWrite(Encoder1ChnA, HIGH); //turn pullup resistor on
  pinMode(Encoder1ChnB, INPUT); digitalWrite(Encoder1ChnB, HIGH); //turn pullup resistor on
  pinMode(EncoderDetent, INPUT); digitalWrite(EncoderDetent, HIGH); //turn pullup resistor on 
  
  PCintPort::attachInterrupt(Encoder1ChnA, &updateEncoder_ISR, CHANGE); 
  PCintPort::attachInterrupt(Encoder1ChnB, &updateEncoder_ISR, CHANGE);
  PCintPort::attachInterrupt(EncoderDetent, &EncoderClick_ISR, FALLING);
  myEncoder.setRate(1.0f);
  myEncoder.setMinMax(63.5,1023);

  Timer1.initialize(150000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
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
  Serial.println("C:\\Users\\Administrator\\Documents\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\IRON_14");

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
//  myEncoder.setPosition(MAX_TEMP-2);
//  encoderPos = MAX_TEMP-2;
#if defined BARGRAPHS  
  verticalBar();
  horizontalBar();
#endif 
  Input = getTemperature();
  Setpoint = encoderPos;
  myPID.SetMode(AUTOMATIC);
}
//=======================================================================
void mainScreen ()
{
  tft.fillScreen(tft.color565(255,255,255));//BLACK    
  tft.setTextSize(2);
  tft.setTextColor(tft.color565(0,255,255));//RED
  tft.setCursor(260,5);
  tft.print(VERSION);
  tft.setTextColor(tft.color565(255,255,0));//BLUE
  tft.setCursor(258,3);
  tft.print(VERSION);

  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(3,57);
  tft.print("TIP");
  tft.setCursor(3,78);
  tft.print("TEMP:");
  tft.setCursor(228,28);
  tft.setTextColor(tft.color565(tempDIV, tempDIV-255, 255-tempDIV));
  tft.print("o");

  tft.setTextColor(tft.color565(255,0,255));
  tft.setCursor(3,143);
  tft.print("SET:");
  tft.setCursor(228,93);
  tft.print("o");

  tft.setTextColor(tft.color565(0,255,255));
  tft.setCursor(3,183);
  tft.print("PWM:");
  tft.setCursor(228,183);
  tft.print("%"); 

  int tempWill = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  myEncoder.setPosition(tempWill);
  tft.setTextColor(tft.color565(255,0,255));  //GREEN  
  tft.setTextSize(2);
  tft.setCursor(100,7);
  tft.print("*MEM = ");
  tft.print(tempWill);
  tft.setTextSize(1);
  tft.print(" o"); 
  tft.setTextSize(2);
  tft.print("C    "); 
#if defined BARGRAPHS  
  verticalBar();
  horizontalBar();
#endif 
}  
//=======================================================================
void loop()
{  
  Input = getTemperature();
  Setpoint = will_temp;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();  
  
  static boolean oneTime = false; 

  will_temp = map(encoderPos, 0, 1024, 0, MAX_TEMP+1);

  int tempWill = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  tft.setTextSize(3);  

  if (digitalRead(STANDBYin) == true)
  {
    tft.setCursor(2,2);
    if(oneTime)
    {      
      tft.fillRect(2, 2, 80, 40, tft.color565(255,255,255));
      beepBuzzer(6250,80);
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
      beepBuzzer(6250,80);
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

  if (digitalRead(STANDBYin) == false)
    standby_act = true;
  else
    standby_act = false;

  if (standby_act && (will_temp >= STANDBY_TEMP ))
    will_temp_tmp = STANDBY_TEMP;

  int MAX_PWM;
  
  actual_temperature = getTemperature();
  if(actual_temperature > MAX_TEMP)
  {    
    pwm = 0;
    digitalWrite(HEAT_LED, LOW);
    actual_temperature = 0; 
    tft.fillScreen(tft.color565(255,255,255));//BLACK
    do
    {
      tft.setTextColor(tft.color565(0,255,255));
      tft.setTextSize(5);
      tft.setCursor(5,80);
      tft.print("UNPLUGGED!");
      delay(200);
      actual_temperature = getTemperature();
    }
    while(actual_temperature > MAX_TEMP);
    
    mainScreen();
    
    tft.setTextSize(7);
    tft.setCursor(100,108);
    tft.setTextColor(tft.color565(255,0,255));//===========================>>>>>>>>>>>>>>>>>>>>>
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

  if(pwm >= 96)
  {
    heater = HIGH;
    tft.fillCircle(150, 190, 11, tft.color565(0,255,255));
  }
  else 
  {    
    heater = LOW;
    tft.fillCircle(150, 190, 11, tft.color565(255,255,255));
  }   writeHEATING(will_temp, Input, pwm);
  //delay(DELAY_MAIN_LOOP);		//wait for some time
}
//=======================================================================
int getTemperature()
{  
  analogWrite(PWMpin, 0);		//switch off heater
  delay(DELAY_MEASURE);			//wait for some time (to get low pass filter in steady state)
  
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(TEMPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) 
  {
    readIndex = 0;
  }
  adcValue = total / numReadings;

#if defined DEBUG_SER  
  Serial.print("ADC Value ");
  Serial.println(adcValue);
#endif  
  analogWrite(PWMpin, pwm);	//switch heater back to last value
  return round(((float) adcValue)*ADC_TO_TEMP_GAIN+ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}
//=======================================================================
void horizontalBar()
{
  for(int i = 0; i <= 4; i++)
  {
    tft.drawFastVLine(i*60, 220, 10, tft.color565(255,0,255));
  }
  tft.drawBargraphHor(0, 207, 241, 14, MAX_TEMP, will_temp, tft.color565(255,0,255), tft.color565(255,255,255));
  tft.setCursor(5,230);
  tft.setTextColor(tft.color565(255,0,255));
  tft.setTextSize(1);
  tft.print("0");
  tft.setCursor(65,230);
  tft.print("100");
  tft.setCursor(125,230);
  tft.print("200");
  tft.setCursor(185,230);
  tft.print("300");
  tft.setCursor(245,230);
  tft.print("400");       
}  
//=======================================================================
void verticalBar()
{
  for(int i = 1; i <= 5; i++)
  {
    tft.drawFastHLine(282, i*43+6, 14, tft.color565(tempDIV, tempDIV-255, 255-tempDIV));
  }    
  tft.drawBargraphVer(270, 49, 14, 173, MAX_TEMP, t1, tft.color565(tempDIV, tempDIV-255, 255-tempDIV), tft.color565(255,255,255));//DO NOT TOUCH!
  tft.setCursor(290,55);
  tft.setTextSize(1);
  tft.setTextColor(tft.color565(tempDIV, tempDIV-255, 255-tempDIV));
  tft.print("400");
  tft.setCursor(290,98);
  tft.print("300");
  tft.setCursor(290,141);
  tft.print("200");
  tft.setCursor(290,184);
  tft.print("100");
  tft.setCursor(290,227);
  tft.print("0");      
}  
//=======================================================================
void writeHEATING(int tempWILL, int tempVAL, int pwmVAL)
{
  static int d_tempWILL = 1;//2		
  static int tempWILL_OLD = 1;//10
  static int tempVAL_OLD = 1;//10
  static int pwmVAL_OLD	= 1;//10

  pwmVAL = map(pwmVAL, 0, 255, 0, 99);

  tft.setTextSize(7);
  if (tempVAL_OLD != tempVAL)
  {
    tft.setCursor(100,40);
    tft.setTextColor(WHITE);

    if ((tempVAL_OLD/100) != (tempVAL/100))
    {
      tft.print(tempVAL_OLD/100);
    }
    else
      tft.print(" ");

    if ( ((tempVAL_OLD/10)%10) != ((tempVAL/10)%10) )
      tft.print((tempVAL_OLD/10)%10 );
    else
      tft.print(" ");

    if ( (tempVAL_OLD%10) != (tempVAL%10) )
      tft.print(tempVAL_OLD%10 );

    tft.setCursor(100,40);
    tft.setTextColor(BLACK);

    if (tempVAL < 100)
      tft.print(" ");
    if (tempVAL <10)
      tft.print(" ");

    tempDIV = round(float(tempWILL - tempVAL) * 8.5);
    tempDIV = tempDIV > 254 ? tempDIV = 254 : tempDIV < 0 ? tempDIV = 0 : tempDIV;
    tft.setTextColor(tft.color565(tempDIV, tempDIV-255, 255-tempDIV));//===========================>>>>>>>>>>>>>>>>>>>>>

    if (standby_act)
    {
      tft.setTextColor(tft.color565(0,255,255));//===========================>>>>>>>>>>>>>>>>>>>>>
    }

    tft.print(tempVAL); 
    t1 = tempVAL;
    
    tft.setTextSize(2);
    tft.setCursor(228,28);
    tft.setTextColor(tft.color565(tempDIV, tempDIV-255, 255-tempDIV));
    tft.print("o");
#if defined BARGRAPHS
    verticalBar();
#endif    
    tempVAL_OLD = tempVAL; 
  }

  if ((tempWILL_OLD+d_tempWILL < tempWILL) || (tempWILL_OLD-d_tempWILL > tempWILL))
  {
    tft.setTextSize(7);
    tft.setCursor(100,108);
    tft.setTextColor(WHITE);

    if ((tempWILL_OLD/100) != (tempWILL/100))
    {
      tft.print(tempWILL_OLD/100);
    }
    else
      tft.print(" ");

    if ( ((tempWILL_OLD/10)%10) != ((tempWILL/10)%10) )
      tft.print((tempWILL_OLD/10)%10 );
    else
      tft.print(" ");

    if ( (tempWILL_OLD%10) != (tempWILL%10) )
      tft.print(tempWILL_OLD%10 );

    tft.setCursor(100,108);
    tft.setTextColor(tft.color565(255,0,255));//===========================>>>>>>>>>>>>>>>>>>>>>
    if (tempWILL < 100)
      tft.print(" ");
    if (tempWILL <10)
      tft.print(" ");

    tft.print(tempWILL);
    t2 = tempWILL;
#if defined BARGRAPHS    
    horizontalBar();
#endif    
    tempWILL_OLD = tempWILL;
  }

  tft.setTextSize(3);
  if (pwmVAL_OLD != pwmVAL)
  {
    tft.setCursor(170,178);
    tft.setTextColor(WHITE); 

    if ((pwmVAL_OLD/100) != (pwmVAL/100))
    {
      tft.print(pwmVAL_OLD/100);
    }
    else
      tft.print(" ");

    if ( ((pwmVAL_OLD/10)%10) != ((pwmVAL/10)%10) )
      tft.print((pwmVAL_OLD/10)%10 );
    else
      tft.print(" ");

    if ( (pwmVAL_OLD%10) != (pwmVAL%10) )
      tft.print(pwmVAL_OLD%10 );

    tft.setCursor(170,178);
    tft.setTextColor(tft.color565(0,255,255));//RED
    if (pwmVAL < 100)
      tft.print(" ");
    if (pwmVAL <10)
      tft.print(" ");

    tft.print(pwmVAL);
    pwmVAL_OLD = pwmVAL;
/*
    if(pwmVAL_OLD != 0)
    {
            //bmpDraw("FILE9.bmp", 130, 167);
//      digitalWrite(HEAT_LED,HIGH); 
      tft.fillCircle(150, 300, 11, tft.color565(0,255,255));
    }
    else 
    {
            //bmpDraw("FILE10.bmp", 130, 167);
//      digitalWrite(HEAT_LED,LOW); 
      tft.fillCircle(150, 300, 11, tft.color565(255,255,255));
    } 
 */   
  }  
}
//=======================================================================
void beepBuzzer(unsigned long hz, unsigned long ms) 
{ 
  unsigned long us = (750000 / hz);  
  unsigned long rep = (ms * 500L) / us; 

  for (int i = 0; i < rep; i++) 
  {  
    digitalWrite(BUZZER_PIN, HIGH);  
    delayMicroseconds(us);  
    digitalWrite(BUZZER_PIN, LOW);  
    delayMicroseconds(us);  
  }
}
//=======================================================
void updateEncoder_ISR() 
{
  rotating = true;
  int increment = 1;
  myEncoder.lowLevelTick();
  encoderPos = myEncoder.getPosition();
  if(encoderPos <= 63.5)
  {
    myEncoder.setPosition(63.5);
    encoderPos = 63.5;
  }  
  if(encoderPos >= 1023) 
  {
    myEncoder.setPosition(1023);//1150
    encoderPos = 1023;
  }  
  beepBuzzer(6250,1);
}
//=======================================================
void EncoderClick_ISR() 
{  
  myEncoder.lowLevelClick();
  {
    EEPROM.writeInt(WILL_TEMP_EEPROM_ADDRESS, will_temp);
    EEPROM.writeInt(ENCODER_EEPROM_ADDRESS, encoderPos);
    myEncoder.setPosition(encoderPos);
    memWrite = true;
    memNoWrite = false;  
    beepBuzzer(6250,80);
  }  
}
//=======================================================
void timer1_ISR()
{
  Timer1.detachInterrupt();
  state =!state;
  switch(heater)
  {
    case HIGH:
      digitalWrite(HEAT_LED, state);
    break;
    case LOW:
       digitalWrite(HEAT_LED, LOW);
    break;  
  }  
  Timer1.attachInterrupt( timer1_ISR );
}
//=======================================================
void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) 
  {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) 
  {
    switch(divisor) 
    {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
//=======================================================================
#define BUFFPIXEL 20

void bmpDraw(char *filename, int x, int y) 
{
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;
  boolean  first = true;

  if((x >= tft.width()) || (y >= tft.height())) return;

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) 
  {
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) 
  {
    // BMP signature
    read32(bmpFile);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) 
    { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel

      if((bmpDepth == 24) && (read32(bmpFile) == 0)) 
      { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) 
        {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) 
        { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
          pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) 
          { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) 
          { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.pushColors(lcdbuffer, lcdidx, first);
                lcdidx = 0;
                first  = false;
              }
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = tft.color565(r,g,b);
          } // end pixel
        } // end scanline
        // Write any remaining data to LCD
        if(lcdidx > 0) 
        {
          tft.pushColors(lcdbuffer, lcdidx, first);
        }

      } // end goodBmp
    }
  }
  bmpFile.close();
  if(!goodBmp) tft.println(F("BMP format not recognized."));
}
//=======================================================================
uint16_t read16(File &f) 
{
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}
//=======================================================================
uint32_t read32(File &f) 
{
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

//=======================================================================

//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================