#include <Arduino.h>

#define SSR_PIN 33  // Solid State Relay Pin

// IMPORTANT: You need to set your SPI pins and display driver. In PlatformIO, you can do this in the platformio.ini file. For Arduino IDE, you need to do it in User_Setup.h in the TFT_eSPI library folder

//  #########################################################################
//  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
//  ######################################################################### 
#include <TFT_eSPI.h> // Add TFT_eSPI by Bodmer to library
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#include "Free_Fonts.h" // Include the header file attached to this sketch

#include <stdint.h>

// Using a display with a different touch chip is easy, just set up your touch as you would normally and replace the next 3 lines with the calls you use to get X, Y and the interrupt pin you are using (that tells you the screen was pressed - take note whether it gets pulled HIGH or LOW because then you might have to change the if statements below i.e. if(digitalRead(INT_N_PIN) != 1)). Then you can uncomment all references to the FT6336U I am using. Also, note below, I switched the x/y to get the proper portrait/landscape orientation
#define GET_X_COORDINATE ft6336u.read_touch1_y()
#define GET_Y_COORDINATE ft6336u.read_touch1_x()
#define INT_N_PIN 25

// You also need to set up the X & Y resolution for your touch screen (the value it returns for X and Y).
// The touch mapping can't usually be rotated and is absolute. Change min/max to invert planes. 
#define TS_MINX 0
#define TS_MINY 320
#define TS_MAXX 480
#define TS_MAXY 0

// You can also use your own thermocouple amp. Just replace the line below with the call you use to get the temperature. Mine returns a float/double in Celsius. If you are getting anything else, add code at the actual call below in the sketch to convert it into C. Note that some amps need to be queried first before they are ready to return the temp. The 31856 in the original code, had a maxthermo.conversionComplete() call that would wait until it said it was ready before returning a value. You will need to add that below if needed. But the READ_THERMOCOUPLE call occurs only once in this sketch. Then you can remove all reference to my amp MCP9600
#define READ_THERMOCOUPLE mcp.readThermocouple()
//MCP9600 stuff
#include <Wire.h>
#include <Adafruit_I2CDevice.h> //I2C stuff can be found in adafruit/Adafruit BusIO
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp;

//Touch includes
#include "FT6336U.h"  // FocalTech FT6336U (Self-Capacitive Touch Panel Controller) library for Arduino. by aselectroworks

// Use default I2C pins for you mcu
#define I2C_SDA 21
#define I2C_SCL 22

#define RST_N_PIN 26
// Touch library stuff
FT6336U ft6336u(I2C_SDA, I2C_SCL, RST_N_PIN, INT_N_PIN);  // For some Arduinos for some reason, creating this instance is only allowed with 2 arguments, using this: FT6336U ft6336u(RST_N_PIN, INT_N_PIN);

const int displayWidth = 480, displayHeight = 320;
const int gridSize = 80;  // Our 320x240 display is 2/3 the size of the 480x320

bool setupMenu = false, editMenu = false, reflowMenu = false;
//const int touchHoldLimit = 500;

uint32_t touchLastMillis = 0;
uint16_t debounceDelay = 100;

unsigned long timeSinceReflowStarted;
unsigned long timeTempCheck = 1000;
unsigned long lastTimeTempCheck = 0;
double preheatTemp = 140, soakTemp = 170, reflowTemp = 220, cooldownTemp = 25;
unsigned long preheatTime = 120000, soakTime = 60000, reflowTime = 60000, cooldownTime = 120000, totalTime = preheatTime + soakTime + reflowTime + cooldownTime;
bool preheating = false, soaking = false, reflowing = false, coolingDown = false, newState = false;
uint16_t gridColor = 0x7BEF;
uint16_t preheatColor = TFT_RED, soakColor = 0xFBE0,   reflowColor = 0xDEE0,   cooldownColor = TFT_BLUE; // colors for plotting
uint16_t preheatColor_d = 0xC000,   soakColor_d = 0xC2E0, reflowColor_d = 0xC600, cooldownColor_d =  0x0018; // desaturated colors

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

#include <PID_v1.h> // Need to add to your library. Search for "PID" by Brett Beauregard
// Specify the links and initial tuning parameters
// Awesome explanation of PID and how to tune the values below if you're overshooting or undershooting temp can be found here: https://www.youtube.com/watch?v=hRnofMxEf3Q&pp=ygULZGlnaWtleSBwaWQ%3D
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// put function declarations here:
int myFunction(int, int);

void printState();
void drawGrid();
void drawButton(int x, int y, int w, int h, uint16_t backgroundColor, uint16_t textColor, String text);
void centerText(int x, int y, int w, int h, uint16_t textColor, String text);
void centerText(int x, int y, int w, int h, int justification, uint16_t textColor, String text);
void centerText(int x, int y, int w, int h, int justification, uint16_t textColor, uint16_t bgTextColor, String text);
void drawSetupMenu();
void drawReflowMenu();
void drawEditMenu(String stage);
int getGridCellX();
int getGridCellY();
String formatTime(unsigned long milliseconds);
void plotDataPoint();
void plotReflowProfile();
/*int  mapTime(int time){
  return map(time,0,totalTime,0,displayWidth);
}*/

/*int mapTemp(int temp){
  return map(temp,0,300,3*gridSize,0);
}*/



void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("Solder Reflow Oven");
  delay(100);
  //tft.begin();  // For original code display
  tft.init();           // Init ST7789 320x240
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(1);

  ft6336u.begin();

    if (! mcp.begin(I2C_ADDRESS)) {
        Serial.println("Sensor not found. Check wiring!");
        while (1);
    }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_K);
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

  //mcp.setAlertTemperature(1, 30);
  //Serial.print("Alert #1 temperature set to ");
  //Serial.println(mcp.getAlertTemperature(1));
  //mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);

  Serial.println(F("------------------------------"));

//PID stuff

  Setpoint = cooldownTemp;
  // tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 1);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN,LOW);
  // pinMode(BACKLIGHT_PIN, OUTPUT);
  // digitalWrite(BACKLIGHT_PIN, HIGH); // We'll just have the backlight on all the time. If you want it to sleep, you can toggle this on/off


}

void loop() {

  digitalWrite(SSR_PIN,LOW);
  // ///* Setup Menu *///
  tft.fillScreen(TFT_BLACK);
  //tft.fillRect(0, 160, 120, 80, TFT_BLUE);
  drawSetupMenu();
  setupMenu = true;
  Serial.println("Setup Menu");
  while(setupMenu){
      //     uint16_t xPos = ft6336u.read_touch1_x();
      // uint16_t yPos = ft6336u.read_touch1_y();
      // Serial.print(xPos); Serial.print(" , "); Serial.print(yPos); Serial.println(")");
    // delay(1000);
    // Serial.println("looping");
      //     Serial.print("FT6336U Touch Event/ID 1: ("); // Returns 1 when the screen is pressed, otherwise returns 0
      // Serial.print(ft6336u.read_touch1_event()); Serial.print(" / "); Serial.print(ft6336u.read_touch1_id()); Serial.println(")");
    //touchpoint = ts.getPoint();
    if(digitalRead(INT_N_PIN) != 1){
      if (millis() - touchLastMillis > debounceDelay) {
        Serial.println("touched");
        int setupMenuXPos = getGridCellX(), setupMenuYPos = getGridCellY();
        Serial.print("Setup menu touch: ("); Serial.print(setupMenuXPos); Serial.print(","); Serial.print(setupMenuYPos); Serial.print(") -> ");
        if(setupMenuYPos < 3){ // Somewhere other than the start button
          editMenu = true;
          bool editingPreheat = false, editingSoak = false, editingReflow = false;
          if(setupMenuXPos < 2 ){ // Somwhere within the preheat zone
            editingPreheat = true;
            tft.fillScreen(preheatColor);
            Serial.println("Edit Preheat Menu");
            drawEditMenu("Preheat");
            centerText(1,0,2,1,2,TFT_WHITE,String(int(preheatTemp)));
            centerText(4,0,2,1,2,TFT_WHITE, formatTime(preheatTime));
          }
          else if(setupMenuXPos > 3 ){// Somwhere within the reflow zone
            editingReflow = true;
            tft.fillScreen(reflowColor);
            Serial.println("Edit Reflow Menu");
            drawEditMenu("Reflow");
            centerText(1,0,2,1, 2,TFT_WHITE,String(int(reflowTemp)));
            centerText(4,0,2,1,2,TFT_WHITE, formatTime(reflowTime));
          }
          else{ // Somwhere within the soak zone
            editingSoak = true;
            tft.fillScreen(soakColor);
            Serial.println("Edit Soak Menu");
            drawEditMenu("Soak");
            centerText(1,0,2,1,2, TFT_WHITE,String(int(soakTemp)));
            centerText(4,0,2,1,2, TFT_WHITE, formatTime(soakTime));
          }
          while(editMenu){// Stay in this loop until the save button is pressed
            //touchpoint = ts.getPoint();
            if(digitalRead(INT_N_PIN) != 1){
              if (millis() - touchLastMillis > debounceDelay) {
                Serial.println("touched");
                int editMenuXPos = getGridCellX(), editMenuYPos = getGridCellY();
                Serial.print("Edit menu touch at ("); Serial.print(editMenuXPos); Serial.print(","); Serial.print(editMenuYPos); Serial.print(") -> ");
                if(editMenuYPos == 1){ // One of the up arrows was pressed
                  if(editMenuXPos < 3){ // The Temp up arrow was pressed
                    Serial.println("Temp up arrow");
                    //tft.fillRoundRect(1*gridSize, 0*gridSize, 2*gridSize, gridSize, 10, TFT_BLACK);
                    if(editingPreheat){
                      if(preheatTemp < 300) {
                        preheatTemp += 10;
                        centerText(1,0,2,1,2,TFT_WHITE, preheatColor, String(int(preheatTemp)));                        
                      }
                    }
                    if(editingSoak){
                      if(soakTemp < 300) {
                        soakTemp += 10;
                        centerText(1,0,2,1,2, TFT_WHITE, soakColor, String(int(soakTemp)));                        
                      }
                    }
                    if(editingReflow){
                      if(reflowTemp < 300) {
                        reflowTemp += 10;
                        centerText(1,0,2,1, 2, TFT_WHITE, reflowColor, String(int(reflowTemp)));                        
                      }
                    }
                  }
                  else{// The Time up arrow was pressed
                    Serial.println("Time up arrow");
                    //tft.fillRoundRect(4*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if(editingPreheat){
                      if(preheatTime < 300000) {
                        preheatTime += 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, preheatColor, formatTime(preheatTime));                        
                      }
                    }
                    if(editingSoak){
                      if(soakTime < 300000) {
                        soakTime += 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, soakColor, formatTime(soakTime));                        
                      }
                    }
                    if(editingReflow){
                      if(reflowTime < 300000) {
                        reflowTime += 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, reflowColor, formatTime(reflowTime));                        
                      }
                    }
                  }
                }
                else if(editMenuYPos == 2){// One of the down arrows was pressed
                  if(editMenuXPos < 3){ // The Temp down arrow was pressed
                    Serial.println("Temp down arrow");
                    //tft.fillRoundRect(1*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if(editingPreheat){
                      if(preheatTemp > 100) {
                        preheatTemp -= 10;
                        centerText(1,0,2,1, 2, TFT_WHITE, preheatColor, String(int(preheatTemp)));                        
                      }
                    }
                    if(editingSoak){
                      if(soakTemp > 100) {
                        soakTemp -= 10;
                        centerText(1,0,2,1, 2, TFT_WHITE, soakColor, String(int(soakTemp)));                        
                      }
                    }
                    if(editingReflow){
                      if(reflowTemp > 100) {
                        reflowTemp -= 10;
                        centerText(1,0,2,1, 2, TFT_WHITE, reflowColor, String(int(reflowTemp)));                        
                      }
                    }
                  }
                  else{// The Time down arrow was pressed
                    Serial.println("Time down arrow");
                    //tft.fillRoundRect(4*gridSize+2, 0*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
                    if(editingPreheat){
                      if(preheatTime > 30000) {
                        preheatTime -= 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, preheatColor, formatTime(preheatTime));                        
                      }
                    }
                    else if(editingSoak){
                      if(soakTime > 30000) {
                        soakTime -= 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, soakColor, formatTime(soakTime));                        
                      }
                    }
                    else if(editingReflow){
                      if(reflowTime > 30000) {
                        reflowTime -= 10000;
                        centerText(4,0,2,1, 2, TFT_WHITE, reflowColor, formatTime(reflowTime));                        
                      }
                    }
                  }
                }
                else if(editMenuYPos == 3){ // Save button was pressed
                  Serial.println("Save button");
                  tft.fillScreen(TFT_BLACK);
                  drawSetupMenu();
                  editMenu = false;
                }
                //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses                
                touchLastMillis = millis();
              }

            }
          }
        }
        else{// Start button was pressed
          Serial.println("Start button");
          setupMenu = false;
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses        
        touchLastMillis = millis();
      }

    }
  }
  Serial.println("Exiting while setup");
  ///* Reflow Menu *///
  tft.fillScreen(TFT_BLACK);
  drawReflowMenu();
  drawButton(0,3,2,1, TFT_GREEN, TFT_WHITE, "Start");
  bool start = false;
  while(!start){
    //touchpoint = ts.getPoint();
    if(digitalRead(INT_N_PIN) != 1){
      if (millis() - touchLastMillis > debounceDelay) {
        if(getGridCellX() <2 && getGridCellY() == 3){
          start = true;
          Serial.println("totalTime: ");
          Serial.println(totalTime);
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses        
        touchLastMillis = millis();
      }
    }
  }
  drawButton(0,3,2,1, TFT_RED, TFT_WHITE, "Stop");
  Serial.println("Reflow Menu");
  unsigned long reflowStarted = millis();
  reflowMenu = true;
  while(reflowMenu){
    timeSinceReflowStarted = millis() - reflowStarted;
    if(timeSinceReflowStarted - lastTimeTempCheck > timeTempCheck){
      lastTimeTempCheck = timeSinceReflowStarted;
      printState();
      // We don't need this conversion check that's available on the 31856. We'll read directly
      // // check for conversion complete and read temperature
      // if (maxthermo.conversionComplete()) {
      //   Serial.print("\tSetpoint:"); Serial.print(Setpoint);
      //   Input = maxthermo.readThermocoupleTemperature();
      //   Serial.print("\tInput:"); Serial.print(Input);
      //   myPID.Compute();
      //   if(Output < 0.5){
      //     digitalWrite(SSR_PIN,LOW);
      //   }
      //   if(Output > 0.5){
      //     digitalWrite(SSR_PIN,HIGH);
      //   }
      //   Serial.print("\tOutput:"); Serial.println(Output);
      //   plotDataPoint();
      // }
      // else {
      //   Serial.println("\tConversion not complete!");
      // }
      // // trigger a conversion, returns immediately
      // maxthermo.triggerOneShot();
            // check for conversion complete and read temperature

        Serial.print("\tSetpoint:"); Serial.print(Setpoint);
        Input = READ_THERMOCOUPLE;
        Serial.print("\tInput:"); Serial.print(Input);
        myPID.Compute();
        if(Output < 0.5){
          digitalWrite(SSR_PIN,LOW);
        }
        if(Output > 0.5){
          digitalWrite(SSR_PIN,HIGH);
        }
        Serial.print("\tOutput:"); Serial.println(Output);
        plotDataPoint();

    }
    if(timeSinceReflowStarted > totalTime){
      Serial.println("exit at(timeSinceReflowStarted > totalTime)");
      reflowMenu = false;
    }
    else if(timeSinceReflowStarted > (preheatTime + soakTime + reflowTime)){ // preheat and soak and reflow are complete. Start cooldown
      if(!coolingDown){
        newState = true;
        preheating = false, soaking = false, reflowing = false, coolingDown = true;
      }
      Setpoint = cooldownTemp;
    }
    else if(timeSinceReflowStarted > (preheatTime + soakTime)){ // preheat and soak are complete. Start reflow
      if(!reflowing){
        newState = true;
        preheating = false, soaking = false, reflowing = true, coolingDown = false;
      }
      Setpoint = reflowTemp;
    }
    else if(timeSinceReflowStarted > preheatTime){ // preheat is complete. Start soak
      if(!soaking){
        newState = true;
        preheating = false, soaking = true, reflowing = false, coolingDown = false;
      }
      Setpoint = soakTemp;
    }
    else{ // cycle is starting. Start preheat
      if(!preheating){
        newState = true;
        preheating = true, soaking = false, reflowing = false, coolingDown = false;
      }
      Setpoint = preheatTemp;
    }
    //touchpoint = ts.getPoint();
    if(digitalRead(INT_N_PIN) != 1){
      if (millis() - touchLastMillis > debounceDelay) {
        if(getGridCellX() < 2 && getGridCellY() == 3){
          Serial.println("exit at stop button");
          reflowMenu = false;
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses        
        touchLastMillis = millis();
      }
    }
  }
  drawButton(0,3,2,1, TFT_GREEN, TFT_WHITE, "Done");
  bool done = false;
  while(!done){
    //touchpoint = ts.getPoint();
    if(digitalRead(INT_N_PIN) != 1){
      if (millis() - touchLastMillis > debounceDelay) {
        if(getGridCellX() < 2 && getGridCellY() == 3){
          done = true;
        }
        //delay(touchHoldLimit); // so holding the button down doesn't read multiple presses        
        touchLastMillis = millis();
      }
    }
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

void printState(){
  String time = formatTime(timeSinceReflowStarted);
  Serial.print("Current time: "); Serial.print(time); Serial.print("\t");
  //tft.fillRoundRect(4*gridSize+2, 3*gridSize+2, 2*gridSize-4, gridSize-4, 10, TFT_BLACK);
    centerText(4,3,2,1,0,TFT_WHITE, TFT_BLACK, time);
  centerText(4,3,2,1,2,TFT_WHITE, TFT_BLACK, String(Input));
  String currentState;
  if(preheating){
    currentState = "Preheating";
  }
  if(soaking){
    currentState = "Soaking";
  }
  if(reflowing){
    currentState = "Reflowing";
  }
  if(coolingDown){
    currentState = "Cool Down";
  }
  Serial.print(currentState);
  if(newState){
    //tft.setFont(&FreeMonoBold12pt7b);
    tft.setFreeFont(FSB12);
    newState = false;
    tft.fillRoundRect(1*gridSize+25, 0*gridSize+2, 3*gridSize-4, gridSize-20, 10, TFT_BLACK);
    centerText(2,0,2,1, 0, TFT_WHITE,currentState);
  }
}

void drawGrid(){
  //tft.setFont();
  tft.setFreeFont(FSB12);
  tft.setTextColor(TFT_WHITE);
  tft.drawRect(0,0,displayWidth,displayHeight-gridSize,gridColor);
  for(int i=1; i<6; i++){
    tft.drawFastVLine(i*gridSize,0,displayHeight-gridSize,gridColor);
  }
  for(int j=1; j<4; j++){
    tft.drawFastHLine(0,j*gridSize,displayWidth,gridColor);
  }

  tft.setTextDatum(TL_DATUM);
  tft.drawString("300", 4, 2);
  //tft.setCursor(4,tft.fontHeight()); tft.print("300");
  tft.drawString("200", 4, 1*gridSize + 2);
  tft.drawString("100", 4, 2*gridSize + 2);

  tft.setCursor(1*gridSize+4,3*gridSize-7-4); tft.print(formatTime(totalTime/6));
  tft.setCursor(2*gridSize+4,3*gridSize-7-4); tft.print(formatTime(2*totalTime/6));
  tft.setCursor(3*gridSize+4,3*gridSize-7-4); tft.print(formatTime(3*totalTime/6));
  tft.setCursor(4*gridSize+4,3*gridSize-7-4); tft.print(formatTime(4*totalTime/6));
  tft.setCursor(5*gridSize+4,3*gridSize-7-4); tft.print(formatTime(5*totalTime/6));

  plotReflowProfile();
}

void drawButton(int x, int y, int w, int h, uint16_t backgroundColor, uint16_t textColor, String text){
  //tft.setFont(&FreeMonoBold12pt7b);
  tft.setFreeFont(FMB18);
  tft.setTextColor(textColor);
/*   if(backgroundColor == TFT_BLACK){
    tft.drawRoundRect(x*gridSize+2, y*gridSize+2, w*gridSize-4, h*gridSize-4, 10, TFT_WHITE);
  }
  else{
    tft.fillRoundRect(x*gridSize+2, y*gridSize+2, w*gridSize-4, h*gridSize-4, 10, backgroundColor);
  } */
  tft.fillRoundRect(x*gridSize+2, y*gridSize+2, w*gridSize-4, h*gridSize-4, 10, backgroundColor);
  if(text == "UP_ARROW"){
    tft.fillTriangle(x*gridSize+(w*gridSize-60)/2, y*gridSize+(h*gridSize-52)/2+52, x*gridSize+(w*gridSize-60)/2+60, y*gridSize+(h*gridSize-52)/2+52, x*gridSize+w*gridSize/2, y*gridSize+(h*gridSize-52)/2, textColor);
  }
  else if(text == "DOWN_ARROW"){
    tft.fillTriangle(x*gridSize+(w*gridSize-60)/2, y*gridSize+(h*gridSize-52)/2, x*gridSize+(w*gridSize-60)/2+60, y*gridSize+(h*gridSize-52)/2, x*gridSize+w*gridSize/2, y*gridSize+(h*gridSize-52)/2+52, textColor);
  }
  else{
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, (x*gridSize)+((w*gridSize)/2), (y*gridSize)+((h*gridSize))/2);
  }
}

void centerText(int x, int y, int w, int h, uint16_t textColor, String text){
  //tft.setFont(&FreeMonoBold12pt7b);
  tft.setFreeFont(FSB18);
  int16_t textBoundX, textBoundY;
  uint16_t textBoundWidth, textBoundHeight;
  //tft.getTextBounds(text,0,0,&textBoundX, &textBoundY, &textBoundWidth, &textBoundHeight);
  tft.setCursor(x*gridSize+(w*gridSize-tft.textWidth(text))/2, y*gridSize+(h*gridSize+tft.fontHeight())/2);
  tft.setTextColor(textColor); tft.print(text);
}

void centerText(int x, int y, int w, int h, int justification, uint16_t textColor, String text){
  tft.setFreeFont(FSB18);
  int16_t textBoundX, textBoundY;
  uint16_t textBoundWidth, textBoundHeight;
  textBoundWidth = tft.textWidth(text);
  textBoundHeight = tft.fontHeight();
  switch(justification){
    case 0: //top justified
      tft.setCursor(x*gridSize+(w*gridSize-textBoundWidth)/2, (y*gridSize+(h*gridSize/2-textBoundHeight)/2+textBoundHeight)-10);
      break;
    case 1: //center justified
      tft.setCursor(x*gridSize+(w*gridSize-textBoundWidth)/2, y*gridSize+(h*gridSize+textBoundHeight)/2);
      break;
    case 2: //bottom justified
      tft.setCursor(x*gridSize+(w*gridSize-textBoundWidth)/2, (y*gridSize+gridSize-(h*gridSize/2-textBoundHeight)/2)-10);
      break;
  }
  tft.setTextColor(textColor); tft.print(text);
}

void centerText(int x, int y, int w, int h, int justification, uint16_t textColor, uint16_t bgTextColor, String text){
  tft.setFreeFont(FSB18);
  int16_t textBoundX, textBoundY;
  uint16_t textBoundWidth, textBoundHeight;
  textBoundWidth = tft.textWidth(text);
  textBoundHeight = tft.fontHeight();
    uint32_t xPos, yPos;
  switch(justification){
    case 0: //top justified
      xPos = x*gridSize+(w*gridSize-textBoundWidth)/2;
      yPos = (y*gridSize+(h*gridSize/2-textBoundHeight)/2+textBoundHeight)-10;
      break;
    case 1: //center justified
      xPos = x*gridSize+(w*gridSize-textBoundWidth)/2;
      yPos = y*gridSize+(h*gridSize+textBoundHeight)/2;
      break;
    case 2: //bottom justified
      xPos = x*gridSize+(w*gridSize-textBoundWidth)/2;
      yPos = (y*gridSize+gridSize-(h*gridSize/2-textBoundHeight)/2)-10;
      break;
  }

  tft.setTextColor(textColor, bgTextColor); 
  uint16_t padding = tft.textWidth("00:00");
  tft.setTextPadding(padding);
  tft.setTextDatum(L_BASELINE);
  tft.drawString(text, xPos, yPos);
}

void drawSetupMenu(){
  tft.setFreeFont(FSB12);
  drawButton(0,0,2,3, preheatColor, TFT_WHITE, "");                  drawButton(2,0,2,3, soakColor, TFT_WHITE, "");                 drawButton(4,0,2,3, reflowColor, TFT_WHITE, "");
  centerText(0,0,2,1, TFT_WHITE, "Preheat");                         centerText(2,0,2,1, TFT_WHITE, "Soak");                        centerText(4,0,2,1, TFT_WHITE, "Reflow");
  centerText(0,1,2,1,0, TFT_WHITE, String(int(preheatTemp)) + " C");        centerText(2,1,2,1,0, TFT_WHITE, String(int(soakTemp)) + " C");       centerText(4,1,2,1,0, TFT_WHITE, String(int(reflowTemp)) + " C");
  centerText(0,1,2,1,2, TFT_WHITE, String(formatTime(preheatTime))); centerText(2,1,2,1,2, TFT_WHITE, String(formatTime(soakTime)));centerText(4,1,2,1,2, TFT_WHITE, String(formatTime(reflowTime)));
  centerText(0,2,2,1,0, TFT_WHITE, "min"); centerText(2,2,2,1,0, TFT_WHITE, "min"); centerText(4,2,2,1,0, TFT_WHITE, "min");
  drawButton(0,3,6,1, TFT_GREEN, TFT_WHITE, "Confirm");
  tft.drawCircle(95,87,4,TFT_WHITE); tft.drawCircle(255,87,4,TFT_WHITE); tft.drawCircle(415,87,4,TFT_WHITE); // These are the degree circles. They are absolute so need to change to scale. The x axis does not correspond perfectly to 2/3 for some reason
}

void drawReflowMenu(){
  tft.setFreeFont(FSB12);
  drawGrid();
  centerText(3,3,1,1,0, TFT_WHITE, "Time: ");
  centerText(3,3,1,1,2, TFT_WHITE, "Temp: ");
  //drawButton(0,3,2,1, TFT_RED, TFT_WHITE, "Stop"); drawButton(0,3,2,1, TFT_RED, TFT_WHITE, "Start");
}

void drawEditMenu(String stage){
  tft.setFreeFont(FSB12);
  centerText(0,0,2,1,0, TFT_WHITE, stage); centerText(0,0,2,1,2, TFT_WHITE, "Temp:   "); drawButton(0,1,3,1, TFT_WHITE, TFT_BLACK, "UP_ARROW"); drawButton(0,2,3,1, TFT_WHITE, TFT_BLACK, "DOWN_ARROW");
  centerText(3,0,2,1,0, TFT_WHITE, stage); centerText(3,0,2,1, 2, TFT_WHITE, "Time:   "); drawButton(3,1,3,1, TFT_WHITE, TFT_BLACK, "UP_ARROW"); drawButton(3,2,3,1, TFT_WHITE, TFT_BLACK, "DOWN_ARROW");
  drawButton(0,3,6,1, TFT_GREEN, TFT_WHITE, "Save");
}

int getGridCellX(){
  //int xpoint = touchpoint.x;
  int xpoint = GET_X_COORDINATE; 
  Serial.print("x resistance: ");Serial.print(xpoint); Serial.print(" ");
  xpoint = map(xpoint,TS_MINX,TS_MAXX,displayWidth-1,0);
  if(xpoint < gridSize)
    return 5;
  else if(xpoint < gridSize * 2)
    return 4;
  else if(xpoint < gridSize * 3)
    return 3;
  else if(xpoint < gridSize * 4)
    return 2;
  else if(xpoint < gridSize * 5)
    return 1;
  else
    return 0;

}

int getGridCellY(){
  //int ypoint = touchpoint.y;
  int ypoint = GET_Y_COORDINATE;
  Serial.print("y resistance: ");Serial.print(ypoint); Serial.print(" ");
  ypoint = map(ypoint,TS_MINY,TS_MAXY,0,displayHeight-1);
  if(ypoint < gridSize)
    return 0;
  else if(ypoint < gridSize * 2)
    return 1;
  else if(ypoint < gridSize * 3)
    return 2;
  else
    return 3;
}

String formatTime(unsigned long milliseconds) {
  // Calculate the number of minutes and seconds
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int minutes = totalSeconds / 60;
  unsigned int seconds = totalSeconds % 60;

  // Format the time as a string with a leading zero if necessary
  String formattedTime = /* (minutes < 10 ? "0" : "") + */ String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);

  return formattedTime;
}

/*int  mapTime(int time){
  return map(time,0,totalTime,0,displayWidth);
}*/

/*int mapTemp(int temp){
  return map(temp,0,300,3*gridSize,0);
}*/

void plotDataPoint(){
  uint16_t color;
  if(preheating){
    color = preheatColor;
  }
  if(soaking){
    color = soakColor;
  }
  if(reflowing){
    color = reflowColor;
  }
  if(coolingDown){
    color = cooldownColor;
  }
  tft.fillCircle(map(timeSinceReflowStarted,0,totalTime,0,displayWidth),map(Input,0,300,3*gridSize,0),2, color);
  //tft.fillCircle(mapTime(timeSinceReflowStarted), mapTemp(Input), 2, color);
}

void plotReflowProfile(){
   int xMin, xMax, amp;
   xMin = 0;
   xMax = map(preheatTime,0,totalTime,0,displayWidth);
   amp = map(preheatTemp,0,300,3*gridSize,0) - map(cooldownTemp,0,300,3*gridSize,0);
  for(int i = 0; i <= (xMax-xMin); i++){
    tft.fillCircle(xMin+i,-amp/2*cos(PI*i/(xMax-xMin))+map(cooldownTemp,0,300,3*gridSize,0)+amp/2,2,preheatColor_d);
  }

  xMin = map(preheatTime,0,totalTime,0,displayWidth);
  xMax = map(preheatTime+soakTime,0,totalTime,0,displayWidth);
  amp = map(soakTemp,0,300,3*gridSize,0) - map(preheatTemp,0,300,3*gridSize,0);
  //amp = 80;
  for(int i = 0; i <= (xMax-xMin); i++){
    tft.fillCircle(xMin+i,-amp/2*cos(PI*i/(xMax-xMin))+map(preheatTemp,0,300,3*gridSize,0)+amp/2,2, soakColor_d);
  }

  xMin = map(preheatTime+soakTime,0,totalTime,0,displayWidth);
  xMax = map(preheatTime+soakTime+reflowTime,0,totalTime,0,displayWidth);
  amp = map(reflowTemp,0,300,3*gridSize,0) - map(soakTemp,0,300,3*gridSize,0);
  //amp = 80;
  for(int i = 0; i <= (xMax-xMin); i++){
    tft.fillCircle(xMin+i,-amp/2*cos(PI*i/(xMax-xMin))+map(soakTemp,0,300,3*gridSize,0)+amp/2,2,reflowColor_d);
  }

  xMin = map(preheatTime+soakTime+reflowTime,0,totalTime,0,displayWidth);
  xMax = map(totalTime,0,totalTime,0,displayWidth);
  amp = map(cooldownTemp,0,300,3*gridSize,0) - map(reflowTemp,0,300,3*gridSize,0);
  //amp = 80;
  for(int i = 0; i <= (xMax-xMin); i++){
    tft.fillCircle(xMin+i,-amp/2*cos(PI*i/(xMax-xMin))+map(reflowTemp,0,300,3*gridSize,0)+amp/2,2, cooldownColor_d);
  }
}







