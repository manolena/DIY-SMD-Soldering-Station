//=======================================================================
// SMD Soldering Station with AVR v1.5
// (c) 2016 Manos Mar <manolena2001@gmail.com>, https://manolena.wordpress.com/2015/12/28/diy-smd-soldering-station-with-avr/
// (c) 2014 Martin Kumm http://www.martin-kumm.de/wiki/doku.php?id=Projects:SMD_Solderstation/
// (c) 2014 MatthiasW  https://debugginglab.wordpress.com/2014/10/30/soldering-station/
//=======================================================================
//    INTERRUPT SERVICE ROUTINES
//=======================================================================
//    ENCODER ISR
//=======================================================================
void updateEncoder_ISR() 
{
  myEncoder.lowLevelTick();
  encoderPos = myEncoder.getPosition();
  if(encoderPos <= MIN_TEMP)
  {
    myEncoder.setPosition(MIN_TEMP);
    encoderPos = MIN_TEMP;
  }  
  if(encoderPos >= MAX_TEMP) 
  {
    myEncoder.setPosition(MAX_TEMP);//1150
    encoderPos = MAX_TEMP;
  }
  beepBuzzer(6250,1);
}
//=======================================================================
//    ENCODER'S DETENT ISR
//=======================================================================
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
//=======================================================================
//    TIMER 1 ISR
//=======================================================================
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
  if (digitalRead(STANDBYin) == LOW) standby_act = true;
  else standby_act = false;
  Timer1.attachInterrupt( timer1_ISR );
}
//=======================================================================
//    END OF FILE
//=======================================================================
