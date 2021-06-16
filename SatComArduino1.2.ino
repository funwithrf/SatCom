/*THE FOLLOWING PROGRAM IS AVAILABLE FOR FREE DISTRIBTION.
CREDIT: KEVIN MATTISON (KM6WUM), CAROLINA BIANCHINI MATTISON, KEN LAUER, OTHERS.
Disclaimer: FOR EDUCATIONAL USE ONLY. USE AT YOUR OWN RISK.
THE SAMPLE CODE IS PROVIDED AS IS AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
SUSTAINED BY YOU OR A THIRD PARTY, HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT ARISING IN ANY WAY OUT OF THE USE 
OF THIS SAMPLE CODE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h> // mag portion of the mag sensor
//#include <Adafruit_ADXL345_U.h> // accel portion of the mag sensor not used

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(67895);
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <timer.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#include <SparkFunLSM9DS1.h>

// Use the LSM9DS1 class to create an object. [imu] can be
// named anything. This sensor is used for the Mag.
LSM9DS1 imu;

////////////////
// I2C Setup //
//////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


// for pitch only sensor
int pitchint = 0;

#define LOGO16_GLCD_HEIGHT 128 
#define LOGO16_GLCD_WIDTH  64 
static const unsigned char PROGMEM logo16_glcd_bmp[] = {

};

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
int x;
int countdisplay;
float one = 0; // serial command Az
float two = 0; // serial command El
int three = 0; // serial command auto or manual mode
int four = 0; // serial command move Az Pos
int five = 0; // serial command move Az Neg
int six = 0; // serial command move Az Pos
int seven = 0; // serial command move Az Neg
int eight = 0; // serial command Az mS move time multiplier
int nine = 0; // serial command El mS move time multiplier

int Elone = 0; //serial1 from uno
int HaltMove = 0; // if error is found, halt flagged
int AzMoving = 0; // keep track of whether the Az should be moving
int ElMoving = 0; // keep track of whether the El should be moving
int BypassCal = 0; // high if bypassing calibration of Az max/min movement

float MoveTimeBaseElPrev = 0;
float deltaEl = 0;
float prevEl = 0;
float deltaTargetEl = 0;

float MoveTimeBaseAzPrev = 0;
float deltaAz = 0;
float prevAz = 0;
float deltaTargetAz = 0;

int StopMoveTrack = 0; // keep track of whether a stop move was requested

float pitch = 0;
int PitchAnalog = 0;

int keeplooping = 0;

int displayArray[20];
int xarray;
int StrAdd;
const int Input22 = 22;    // the pin that the first input to the arduino is called out
const int Input23 = 23;    // the pin that the first input to the arduino is called out
const int Output36 = 36;   // the pin that the first input to the arduino is called out
const int Output37 = 37;   // the pin that the first input to the arduino is called out
const int Output38 = 38;   // the pin that the first input to the arduino is called out
const int Output39 = 39;   // the pin that the first input to the arduino is called out
const int Output40 = 40;   // the pin that the first input to the arduino is called out
const int Output41 = 41;   // the pin that the first input to the arduino is called out


int Input22State = 0; //current state of the input
int Input23State = 0; //current state of the input
int Output36State = 0;
int Output37State = 0; 
int Output38State = 0;
int Output39State = 0;
int Output40State = 0;
int Output41State = 0;

String NumberString;
String ProgStatus = "Begin";

float curAz = 0;
float curEl = 0;
float curCompass = 0;
float convEl = 0;
int Mode = 0;
float AzTarget = 0;
float ElTarget = 0;
float AzTargetPrev = 0;
float ElTargetPrev = 0;
int ReadSerial = 0;

int MoveTimeBase = 0;
int MoveTimeBaseEl = 0;
int absmovetime = 0;
float result = 0;
float headingDegrees = 0;

// current sensors
int AzCurrentPosPin = A6;
int AzCurrentNegPin = A7;
int ElCurrentPosPin = A8;
int ElCurrentNegPin = A9;
int AzCurrentPos = 0;
int AzCurrentNeg = 0;
int ElCurrentPos = 0;
int ElCurrentNeg = 0;

// calibration limits
int CWLimit = 11111;
int CCWLimit = 11111;
int CalInProgress = 0;
int CWMap = 11111;
int CCWMap = 11111;
int MappedReading = 11111;
int MappedTarget = 11111;
int ZeroCrossing = 0;
int Gap = 0;
float Range = 0;

// timers create
auto timer = timer_create_default(); // create a timer with default settings
auto timerEl = timer_create_default(); // create a timer with default settings
auto timerCal = timer_create_default(); // create calibration limits timer with default settings
auto timerCalback = timer_create_default(); // create calibration backout timer with default settings
Timer<> default_timer; // save as above

// create a timer that can hold 1 concurrent task, with microsecond resolution
Timer<1, micros> u_timer;
Timer<1, micros> u_timerEl;
Timer<1, micros> u_timerCal; //calibrate limits timer but be careful to take steps so momentum doesn't build up during long moves
Timer<1, micros> u_timerCalback; //move the motor a bit to move out of the cal limit
int movetime = 0;
int movetimeEl = 0;
int movetimeCal = 30000;
int movetimeCalback = 2000;
int calculatedmovetime = 0;

int viewDiag = 1;



void displaySensorDetailsMag(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}







void setup()   {           

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

  // end pitch only sensor
  
  #ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif     
  Serial.begin(9600);
  /* Initialise the sensor */
  



/* Set the range to whatever is appropriate for your project */
  // accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
//   accel.setRange(ADXL345_RANGE_2_G);
  

// display magnetic setup
if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetailsMag();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  //delay(1000);

  // Clear the buffer.
  display.clearDisplay();

    // big bitmap display
  display.drawBitmap(0, 0, logo16_glcd_bmp, 128, 64, 1);
  display.display();
  //delay(1000);
display.clearDisplay();
  
  // draw a bitmap icon and 'animate' movement
 // testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);

//initialize input pins as inputs:
//these are configured as pullups using the internal 20kohm resistor on the board.
//wiring must be changed where you will send GND to each input.
 pinMode(Input22, INPUT_PULLUP);
 pinMode(Input23, INPUT_PULLUP);
 pinMode(Output36, OUTPUT);
 pinMode(Output37, OUTPUT);
 pinMode(Output38, OUTPUT);
 pinMode(Output39, OUTPUT);
 pinMode(Output40, OUTPUT);
 pinMode(Output41, OUTPUT);
 digitalWrite(Output36, HIGH); // Az Neg // HIGH MEANS OPEN RELAY OUT
 digitalWrite(Output37, HIGH); // Az Pos
 digitalWrite(Output38, HIGH); // El Neg
 digitalWrite(Output39, HIGH); // El Pos
 digitalWrite(Output40, HIGH); // LED Arduino Program OK
 digitalWrite(Output41, HIGH); // Beam Moving
}



/////////////////////////////////END SETUP///////////////////////////////////////

void loop() {

 AzCurrentPos = analogRead(AzCurrentPosPin);
 AzCurrentNeg = analogRead(AzCurrentNegPin);
 ElCurrentPos = analogRead(ElCurrentPosPin);
 ElCurrentNeg = analogRead(ElCurrentNegPin);

// if keeplooping is cleared, then check the Az and El
if (keeplooping == 0){
 CheckAzEl();
}

// if keeplooping high then keep moving
if (keeplooping == 1){
  Moving();
}

if (CWLimit == 11111){
  CalLimits();
}
 
 Input22State = digitalRead(Input22);
 Input23State = digitalRead(Input23);
 digitalWrite(Output40, HIGH); // LED Arduino Program OK


if (four == 1) {
 MoveCalc(); 
}
if (five == 1) {
 MoveCalc(); 
}
if (six == 1) {
 MoveCalc(); 
}
if (seven == 1) {
 MoveCalc();  
}

//Read USB data
 ReadSerialData();

// time for Az Move
// now calculated in calibration routine
//MoveTimeBase = eight;

// time for El Move
// now calculated in calibration routine
//MoveTimeBaseEl = nine;

// check for valid data that the beam can safely move to
if (one < 0){
//  one = AzTargetPrev;
//one = 1;
  ProgStatus = "oneissue1";
  DisplayUpdate();
delay(100);
}
if (one > 359){
 // one = AzTargetPrev;
 //one = 359;
  ProgStatus = "oneissue2";
  DisplayUpdate();
delay(100);
}

if (two < 1){
  two = ElTargetPrev;
}
if (two > 89){
  two = ElTargetPrev;
}
if (AzTarget < 1){
  AzTarget = 0;
}
if (ElTarget < 1){
  ElTarget = 0;
}

NewTarget();
}

/////////////////////////////END LOOP///////////////////////////////


// check to see if new target for Az El commanded
void NewTarget() {

/////////////////////////////////////////////////////////////////////////
 // If new Az target, Move Az
if (Mode == 1){
if ((AzTarget != AzTargetPrev) && (ElMoving == 0)){
// keep this here?
//delete this code below after debug

ProgStatus = AzTarget;
DisplayUpdate();
delay(100);
ProgStatus = AzTargetPrev;
DisplayUpdate();
delay(100);
// if zero is included in the Az calibration then map points accordingly
if (ZeroCrossing == 1){

// if Az Target command is outside of calibrated range and zero crossing is calibrated
if(one > CWLimit && one < CCWLimit){
AzTargetPrev = one;
  ProgStatus = "OUTBounds";
  if (viewDiag == 1) {
  Serial.print("AzTarget Out of Bounds check");
  DisplayUpdate();
delay(100);
  StopMove();
}
}
  
// Also need to convert target requests over serial to a map

if (AzTarget < CWLimit || AzTarget == 0){
  MappedTarget = map(AzTarget,0,CWLimit,360-CCWLimit,360-CCWLimit+CWLimit); //CWMap Target Azimuth
  AzTarget = MappedTarget;
   if (viewDiag == 1) {
  Serial.print("AzTarget mapped2b: ");
Serial.println(AzTarget);
ProgStatus = "mapped2b";
DisplayUpdate();
delay(100);
   }
}
else if (AzTarget > CWLimit){
  MappedTarget = map(AzTarget,CCWLimit,359,0,359-CCWLimit); //CCWMap Target Azimuth
  AzTarget = MappedTarget;
   if (viewDiag == 1) {
Serial.print("AzTarget mapped1a: ");
Serial.println(AzTarget);
ProgStatus = "mapped1a";
DisplayUpdate();
delay(100);
  }
}
} //end zero crossing is 1 mapping

// if zero is included in the Az calibration then map points accordingly
if (ZeroCrossing == 0){

// if Az Target command is outside of calibrated range and zero crossing is calibrated
if(one > CWLimit || one < CCWLimit){
AzTargetPrev = AzTarget;
  ProgStatus = "OUTBounds";
  if (viewDiag == 1) {
  Serial.print("AzTarget Out of Bounds check no 0 cross");
  DisplayUpdate();
delay(100);
  StopMove();
}
}
}// end check for non zero crossing
  AzMoving = 1;
  AzTargetPrev = AzTarget;
  ProgStatus = "MovingAz";
  DisplayUpdate();
delay(100);
   if (viewDiag == 1) {Serial.println("Az first time run MoveCalc");
   }
  MoveCalc();
  }

if ((AzMoving > 0) && (AzMoving < 3) && (StopMoveTrack == 1)){
  if (viewDiag == 1) {Serial.println("Az run MoveCalc");
  }
  ProgStatus = "MoveAzG";
  DisplayUpdate();
delay(100);
  MoveCalc();
}

if (AzMoving >= 2){
  AzMoving = 0;
  if (viewDiag == 1) {Serial.println("Az Max corrections now to 0");
  }
  ProgStatus = "MoveAzD";
  DisplayUpdate();
delay(100);
  StopMove();
}
// If new El target, Move El
if ((ElTarget != ElTargetPrev) && (AzMoving == 0)){
  ElMoving = 1;
  ElTargetPrev = ElTarget;
  ProgStatus = "MovingEl";
  DisplayUpdate();
delay(100);
 if (viewDiag == 1) { Serial.println("El first time run MoveCalc");
 }
  MoveCalc();
  }

if ((ElMoving > 0) && (ElMoving < 2) && (StopMoveTrack == 1)){
  if (viewDiag == 1) {Serial.println("El run MoveCalc");
  }
  MoveCalc();
  }

if (ElMoving >= 2){
  ElMoving = 0;
  if (viewDiag == 1) {Serial.println("El Max corrections now to 0");
  }
  StopMove();
}

}

}

/////////////////////////////////////////////////////////////////////

void DisplayUpdate() {

//******************LCD DISPLAY UPDATE********************
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Az:");
  display.println(one); 
  display.print("El:");
  display.println(two); 
  display.print(ProgStatus);
  display.println(" ");
  display.print(curCompass);display.print(" ");
  display.println(curEl); 
  display.display(); 
  
}

// calculates how many degrees needed to move, the direction, and how much time needed to power motor to get there.

void MoveCalc() {
  //AzTargetPrev = AzTarget; //does this Need to be here and repeated elsewhere troubleshoot
delay(100); // settle time.
CheckAzEl();
delay(100); // settle time.
if (Mode == 0) {
  movetimeEl = 5000; // time in mS
timerEl.in(movetimeEl, StopMove);

if (four == 1) {
  Serial.println("Az Man Move Pos");
 AzMovePos(); 
}
if (five == 1) {
  Serial.println("Az Man Move Neg");
 AzMoveNeg(); 
}
if (six == 1) {
  Serial.println("El Man Move Pos");
 ElMovePos(); 
}
if (seven == 1) {
  Serial.println("El Man Move Neg");
 ElMoveNeg(); 
}
  
}

// If in automatic mode, decide which way to move
if (Mode == 1) {

if (ElMoving != 0) {
if (viewDiag == 1) {
Serial.println("curEl at: ");
      Serial.print(curEl);
}
StopMoveTrack = 0;

// Max Elevation the unit can handle is 80 degrees.
if (ElTarget > 79){
ElTarget = 78;
}

// Elevation currently reading should always be above horizon or something is very wrong.
if (curEl < 0) {
  StopMove();
}

if (ElMoving == 1) {
  MoveTimeBaseElPrev = MoveTimeBaseEl;
  prevEl = curEl;
}

// calculate how long each El step took and calculate new time base for next move
///////// calculation moved to initial calibration routine. same time used thereafter.
////////////////////////////////////////////////////////////////////////////////////
if (ElMoving > 1) {
  
  deltaEl = curEl - prevEl;
  if (deltaEl < 0) {
    deltaEl = prevEl - curEl;
  }
  if (deltaEl == 0) {
    deltaEl = 1;
  }
  MoveTimeBaseEl = movetimeEl / deltaEl;

  MoveTimeBaseElPrev = MoveTimeBaseEl;
  prevEl = curEl;
  
}

// check for calculation outcome being very wrong
if (MoveTimeBaseEl > 5000) {
  MoveTimeBaseEl = 1000;
}
if (MoveTimeBaseEl < 1) {
  MoveTimeBaseEl = 1000;
}

// Move Elevation +
if ((curEl < ElTarget + 1) && (curEl > ElTarget - 1)) {
 ElTargetPrev = ElTarget;
      ElMoving = 0;
      StopMoveTrack = 0;
      if (viewDiag == 1) {
      Serial.println("Corrected El within tolerance");
      }
}
else if (curEl < ElTarget + 1){
  if (viewDiag == 1) {
  Serial.println("curEl less than target at: ");
      Serial.print(ElTarget);
  }
      ElTargetPrev = ElTarget;
      movetimeEl = (ElTarget - curEl) * MoveTimeBaseEl;
      if (movetimeEl < 0){
      movetimeEl =  (curEl - ElTarget) * MoveTimeBaseEl; 
      }
      timerEl.in(movetimeEl, StopMove);
      ElMoving++;
      if (viewDiag == 1) {
      Serial.println("Send to El Move Pos Routine");
      Serial.println(movetimeEl);
      }
  ElMovePos();
 }
 // Move Elevation -
else if (curEl > ElTarget + 1){
  
      ElTargetPrev = ElTarget;
      movetimeEl = (curEl - ElTarget) * MoveTimeBaseEl;
       if (movetimeEl < 0){
      movetimeEl =  (curEl - ElTarget) * MoveTimeBaseEl; 
      }
      timerEl.in(movetimeEl, StopMove);
      ElMoving++;
      if (viewDiag == 1) {
      Serial.println("curEl greater than target at: ");
      Serial.print(ElTarget);
      Serial.println("Send to El Move Neg Routine from movecalc");
      Serial.println(movetimeEl);
      }
  ElMoveNeg();
    }
  
 else {
      ElTargetPrev = ElTarget;
      ElMoving = 0;
      StopMoveTrack = 0;
      if (viewDiag == 1) {
      Serial.println("Corrected El after all checks tolerance");
      }
      }
 }



if (AzMoving != 0) {
  if (viewDiag == 1) {
Serial.println("curAz at: ");
      Serial.print(curAz);
  }
StopMoveTrack = 0;

if (AzMoving == 1) {
  MoveTimeBaseAzPrev = MoveTimeBase;
  prevAz = curAz;
  if (viewDiag == 1) {
  Serial.print("entering azmoving is 1 routine ");
  }
}

// calculate how long each Az step took and calculate new time base for next move
if (AzMoving > 1) {
  if (viewDiag == 1) {
Serial.print("entering azmoving greater than 1 routine ");
  }
  deltaAz = curAz - prevAz;
  
  if (deltaAz < 0) {
    deltaAz = prevAz - curAz;
  }
  if (deltaAz == 0) {
    deltaAz = 1;
  }
  
  MoveTimeBase = movetime / deltaAz;

  MoveTimeBaseAzPrev = MoveTimeBase;
if (viewDiag == 1) {
  Serial.print("curAz: ");
  Serial.println(curAz);
    Serial.print("prevAz: ");
  Serial.println(prevAz);
}
  
  prevAz = curAz;
  if (viewDiag == 1) {
    Serial.print("DeltaAz: ");
  Serial.println(deltaAz);
    Serial.print("MoveTimeBase: ");
  Serial.println(MoveTimeBase);
  }
  
}

// check for calculation outcome being very wrong
if (MoveTimeBase > 10000) {
  MoveTimeBase = 100;
}
if (MoveTimeBase < 1) {
  MoveTimeBase = 100;
}


 if ((curAz < AzTarget + 1) && (curAz > AzTarget - 1)) {
  AzTargetPrev = AzTarget;
      AzMoving = 0;
      StopMoveTrack = 0;
      if (viewDiag == 1) {
      Serial.println("Corrected Az corrected within tolerance");
      }
}

// Move CW +
else if (curAz < AzTarget + 1){
      AzTargetPrev = AzTarget;
      movetime = (AzTarget - curAz) * MoveTimeBase;
      if (movetime < 0){
         movetime = (curAz - AzTarget) * MoveTimeBase;
      }
      timer.in(movetime, StopMove);
      if (viewDiag == 1) {
      Serial.println("curAz less than target at: ");
      Serial.print(AzTarget);
      Serial.println("Send to Az Move Pos Routine");
      Serial.println(movetime);
      }
      AzMoving++;
  AzMovePos();
  }

// Move CCW -
else if (curAz > AzTarget - 1){
      AzTargetPrev = AzTarget;
      movetime = (curAz - AzTarget) * MoveTimeBase;
      if (movetime < 0){
        movetime = (curAz - AzTarget) * MoveTimeBase;
      }
      timer.in(movetime, StopMove);
      if (viewDiag == 1) {
      Serial.println("curAz greater than target at: ");
      Serial.print(AzTarget);
      Serial.println("Send to Az Move Neg Routine");
      Serial.println(movetime);
      }
      AzMoving++;
  AzMoveNeg();
    }
 
else {
      AzTargetPrev = AzTarget;
      AzMoving = 0;
      StopMoveTrack = 0;
      if (viewDiag == 1) {
      Serial.println("Corrected Az after all options");
      }
      }
  }
}
}


void AzMovePos() { // Move CW
 digitalWrite(Output37, HIGH); // Az Pos
 digitalWrite(Output36, LOW); // Az Neg
 digitalWrite(Output39, HIGH); // El Pos
 digitalWrite(Output38, HIGH); // El Neg
 digitalWrite(Output41, LOW); // Beam Moving
 four = 0;
 if (viewDiag == 1) {
 Serial.println("Az Positive Routine"); 
 }
 keeplooping = 1;
 Moving();
}
void AzMoveNeg() { // Move CCW
 digitalWrite(Output37, LOW); // Az Pos
 digitalWrite(Output36, HIGH); // Az Neg
 digitalWrite(Output39, HIGH); // El Pos
 digitalWrite(Output38, HIGH); // El Neg
 digitalWrite(Output41, LOW); // Beam Moving
 five = 0;
 if (viewDiag == 1) {
 Serial.println("Az Negative Routine"); 
 }
 keeplooping = 1;
 Moving();
}
void ElMovePos() {
 digitalWrite(Output37, HIGH); // Az Pos
 digitalWrite(Output36, HIGH); // Az Neg
 digitalWrite(Output39, LOW); // El Pos
 digitalWrite(Output38, HIGH); // El Neg
 digitalWrite(Output41, LOW); // Beam Moving
 six = 0;
 if (viewDiag == 1) {
 Serial.println("El Positive Routine"); 
 }
 keeplooping = 1;
 Moving();
}
void ElMoveNeg() {
 digitalWrite(Output37, HIGH); // Az Pos
 digitalWrite(Output36, HIGH); // Az Neg
 digitalWrite(Output39, HIGH); // El Pos
 digitalWrite(Output38, LOW); // El Neg
 digitalWrite(Output41, LOW); // Beam Moving
 seven = 0;
 if (viewDiag == 1) {
 Serial.println("El Negative Routine"); 
 }
 keeplooping = 1;
 Moving();
}

void StopMove() {
 digitalWrite(Output37, HIGH); // Az Pos
 digitalWrite(Output36, HIGH); // Az Neg
 digitalWrite(Output39, HIGH); // El Pos
 digitalWrite(Output38, HIGH); // El Neg
 digitalWrite(Output41, HIGH); // Beam Not moving output indicator
 four = 0;
 five = 0;
 six = 0;
 seven = 0;
 if (viewDiag == 1) {
 Serial.println("StopMove Routine Complete"); 
 }
 StopMoveTrack = 1;
 keeplooping = 0;
 // If Az has tried to correct more than 1 times, stop attempting to move
if (AzMoving > 2){
  AzMoving = 0;
  StopMoveTrack = 0;
  if (viewDiag == 1) {
  Serial.println("Max AzMoving in StopMove Routine");
  }
  }
 if (ElMoving > 2){
  ElMoving = 0;
  StopMoveTrack = 0;
  if (viewDiag == 1) {
  Serial.println("Max ElMoving in StopMove Routine");
  }
  }
  delay(100);
ProgStatus = "StopMove";
 CheckAzEl();

}

void Moving() {

//Read USB to keep buffer from filling
  ReadSerialData();

// check for excessive current draw
  if (AzCurrentPos > 545) {
    if (viewDiag == 1) {
Serial.println("Az Current Pos Draw Stop: ");
Serial.print(ElCurrentPos);
    }
AzMoving = 0;
StopMove();
}

  if (AzCurrentNeg > 545) {
    if (viewDiag == 1) {
Serial.println("Az Current Neg Draw Stop: ");
Serial.print(ElCurrentNeg);
    }
AzMoving = 0;
StopMove();
}

if (ElCurrentPos > 545) {
  if (viewDiag == 1) {
Serial.println("El Current Pos Draw Stop: ");
Serial.print(ElCurrentPos);
  }
ElMoving = 0;
StopMove();
}

if (ElCurrentNeg > 545) {
  if (viewDiag == 1) {
Serial.println("El Current Neg Draw Stop: ");
Serial.print(ElCurrentNeg);
  }
ElMoving = 0;
StopMove();
}
     timer.tick(); // tick the timer
  u_timer.tick();

     timerEl.tick(); // tick the timer
  u_timerEl.tick();

  timerCal.tick(); // tick the timer
  u_timerCal.tick();

  timerCalback.tick(); // tick the timer
  u_timerCalback.tick();

  // remember to set keeplooping to 0 get out of moving loop
if (keeplooping == 1){

Moving();
}
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
pitchint = (int) pitch;
if (pitchint < 0) {
  pitchint = 0;
}
}


void CheckAzEl() {
   // if there's any serial available, read it:
 /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
//float xoffset = 0.02;
//float yoffset = -7;

//working early 2021.
//float xoffset = 0.45;
//float yoffset = -7.36;

float xoffset = -0.73;
float yoffset = -7.23;

float heading = atan2(event.magnetic.y - yoffset, event.magnetic.x - xoffset);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mountain View CA was: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // Fargo ND is: +4.71 degrees or 0.08 radian
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.08;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI;
  
 // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  curAz = (int)headingDegrees;
  curCompass = curAz; //track actual Azimuth if in zerocrossing
// If calibration has a zero degree crossing then remap the current Azimuth to new coordinates
  if (ZeroCrossing == 1){
  
if (curAz > CWLimit){
MappedReading = map(curAz,CCWLimit,359,0,359-CCWLimit); //CCWMap Compass Reading
curAz = MappedReading;
}

else if (curAz < CWLimit){
MappedReading = map(curAz,0,CWLimit,360-CCWLimit,360-CCWLimit+CWLimit); //CWMap Compass Reading
curAz = MappedReading;
}
  }
 // sensors_event_t event; 
//  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
 convEl = abs(event.acceleration.y);
 result = map(convEl, 0, 10, 0, 9000);

 // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  printAttitude(imu.ax, imu.ay, imu.az, 
                 -imu.my, -imu.mx, imu.mz);
  curEl = pitch;

DisplayUpdate();
 //Serial.println("Check Az El Complete");
 //delay(2000);
}

void ReadSerialData() {
// *****************USB serial port****************************

  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    one = Serial.parseFloat();
    // do it again:
    two = Serial.parseFloat();
    // do it again:
    three = Serial.parseInt();
    // do it again:
    four = Serial.parseInt();
    // do it again:
    five = Serial.parseInt();
    // do it again:
    six = Serial.parseInt();
    // do it again:
    seven = Serial.parseInt();
    // and again:
    eight = Serial.parseInt();
    // and again:
    nine = Serial.parseInt();
    
    
    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      
      Serial.print(one);
      Serial.print(two);
      Serial.print(three);
      Serial.print(four);
      Serial.print(five);
      Serial.print(six);
      Serial.print(seven);
      Serial.print(eight);
      Serial.print(nine);
      
    }// end reading carriage return serial data
    
    //Populate mapped serial data to targets and modes
    AzTarget = one;
    ElTarget = two;
    Mode = three;
}
DisplayUpdate();
}

void CalLimits() {

if (BypassCal == 0){
  // declare calibration in progress
CalInProgress == 1;
ProgStatus = "CalAzNeg";
DisplayUpdate();

// Now Move Az counterclockwise all the way to the hard limit in the motor circuit
timerCal.in(movetimeCal, StopMove);
AzMoveNeg();
CheckAzEl();
CCWLimit = curAz;
ProgStatus = "CalAzPos";
DisplayUpdate();
// Now Move Az clockwise all the way to the hard limit in the motor circuit
timerCal.in(movetimeCal, StopMove);
AzMovePos();
CheckAzEl();
CWLimit = curAz;
// Move Az neg to back off of hard limit. 
// and calculate move time per degree of Az move.
prevAz = curAz;
ProgStatus = "CalAzClc";
DisplayUpdate();
timerCalback.in(movetimeCalback, StopMove);
AzMoveNeg();
delay(100);
CheckAzEl();

deltaAz = curAz - prevAz;

if (curAz > 330 && prevAz < 30) {
  deltaAz = 360-curAz + prevAz;
}
  
  if (deltaAz < 0) {
    deltaAz = prevAz - curAz;
  }
  if (deltaAz == 0) {
    deltaAz = 1;
  }
  
  MoveTimeBase = movetimeCalback / deltaAz;
ProgStatus = "TimeAz";
DisplayUpdate();
delay(100);
ProgStatus = MoveTimeBase;
DisplayUpdate();
delay(1000);
} // end if for checking whether bypass cal disabled
// if wanting to bypass Az limit calibration set them manually as listed below
if (BypassCal == 1){
  CCWLimit = 217;
  CWLimit = 146;
}
// Check if gap for rough idea on calibration success
Gap = abs(CCWLimit-CWLimit);

ProgStatus = "CalGood";
// If gap is larger than expected or less than expected flag calibration as failed
 if (Gap < 10){
 CalInProgress = 5; // Failed calibration 
 ProgStatus = "CalFail";
 DisplayUpdate();
 delay(100);
 }

// Check for 0 degrees being included in calibration range
if (abs(CCWLimit-CWLimit) < 100){
 ZeroCrossing = 1;  // decide on mapping based on this
}
else {
 ZeroCrossing = 0;  // decide on mapping based on this 
}


// calculate the time it takes for Elevation degrees chnage per second of move time
  CheckAzEl();
  ProgStatus = "CalElClc";
DisplayUpdate();
  prevEl = curEl; // the elevation read right now will be compared to the elevation read after a second
timerCalback.in(movetimeCalback, StopMove);
ElMovePos();
delay(100);
CheckAzEl();
  deltaEl = curEl - prevEl;
  if (deltaEl < 0) {
    deltaEl = prevEl - curEl;
  }
  if (deltaEl == 0) {
    deltaEl = 1;
  }
  // result calculated amount of move time per degree
  MoveTimeBaseEl = movetimeCalback / deltaEl;

 // MoveTimeBaseElPrev = MoveTimeBaseEl;
 // prevEl = curEl;
ProgStatus = "TimeEl";
DisplayUpdate();
delay(100);
ProgStatus = MoveTimeBaseEl;
DisplayUpdate();
delay(100);
ProgStatus = "CalElAdj";
DisplayUpdate();
CheckAzEl();
if (curEl < 0) {
  ElTarget = 10;
      movetimeEl = (ElTarget - curEl) * MoveTimeBaseEl;
  timerEl.in(movetimeEl, StopMove);
  ElMovePos();
}
if (curEl > 15) {
    ElTarget = 10;
      movetimeEl = (curEl - ElTarget) * MoveTimeBaseEl;
  timerEl.in(movetimeEl, StopMove);
  ElMoveNeg();
}
ProgStatus = "CalDone";
DisplayUpdate();
 // Print to serial if in diag mode
if (viewDiag == 1) {
  Serial.println("Gap:");
  Serial.println(Gap);
  Serial.print("CCWLimit:");
  Serial.println(CCWLimit);
  Serial.print("CWLimit:");
  Serial.println(CWLimit);
  Serial.print("Status:");
  Serial.println(ProgStatus);
  Serial.print("ZeroCross:");
  Serial.println(ZeroCrossing);
    Serial.print("MoveTimeBase:");
  Serial.println(MoveTimeBase);
    Serial.print("MoveTimeBaseEl:");
  Serial.println(MoveTimeBaseEl);
  }
}
