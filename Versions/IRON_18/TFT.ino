//=======================================================================
// SMD Soldering Station with AVR v1.5
// (c) 2016 Manos Mar <manolena2001@gmail.com>, https://manolena.wordpress.com/2015/12/28/diy-smd-soldering-station-with-avr/
// (c) 2014 Martin Kumm http://www.martin-kumm.de/wiki/doku.php?id=Projects:SMD_Solderstation/
// (c) 2014 MatthiasW  https://debugginglab.wordpress.com/2014/10/30/soldering-station/
// (c) 2016 FireDeveloper https://github.com/FireDeveloper : Special thanks to Stefanos for his RGB MultyBit Calculator
//=======================================================================
//    TFT FUNCTIONS
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
  }  
}
//=======================================================================
//    END OF FILE
//=======================================================================
