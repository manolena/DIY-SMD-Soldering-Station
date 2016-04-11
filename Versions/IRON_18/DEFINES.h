//=======================================================================
// SMD Soldering Station with AVR v1.5
// (c) 2016 Manos Mar <manolena2001@gmail.com>, https://manolena.wordpress.com/2015/12/28/diy-smd-soldering-station-with-avr/
// (c) 2014 Martin Kumm http://www.martin-kumm.de/wiki/doku.php?id=Projects:SMD_Solderstation/
// (c) 2014 MatthiasW  https://debugginglab.wordpress.com/2014/10/30/soldering-station/
//=======================================================================
#ifndef DEFINES_H
#define DEFINES_H
//=======================================================================
//    DEFINITIONS
//=======================================================================
#define MCUFRIEND_2_4_TFT  0x9341

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

#define ADC_TO_TEMP_GAIN 	1.8//0.99//2.50//0.53 //0.415
#define ADC_TO_TEMP_OFFSET      25.0
#define STANDBY_TEMP		175
#define MAX_TEMP		450
#define MIN_TEMP	        25 // Minimum setpoint temperature

#define MAX_PWM_LOW		180//180
#define MAX_PWM_HI		240//210		

#define PWM_DIV                 1024		

#define WILL_TEMP_EEPROM_ADDRESS 0x10
#define ENCODER_EEPROM_ADDRESS   0x20

//#define DEBUG_SER
#define BARGRAPHS
//=======================================================================
//    VARIABLES
//=======================================================================
//PID parameters
double Setpoint, Input, Output;
double aggKp=8.00, aggKi=0.10, aggKd=4.00;
double consKp=4.00, consKi=0.05, consKd=2.00;
//=======================================================================
int pwm = 0; //pwm Out Val 0.. 255
unsigned int actual_temperature, will_temp = STANDBY_TEMP;
int MAX_PWM;
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

#endif
//=======================================================================
//    END OF FILE
//=======================================================================
