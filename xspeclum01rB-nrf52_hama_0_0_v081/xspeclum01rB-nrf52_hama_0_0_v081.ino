/*
 * Program for 
 * Adafruit Feather nRF52840 with XSPECLUM01 board.
 * 
 * Spectral sensor:
 * - HAMAMATSU c12880ma
 * GX16 Sensor(s):
 * - None
 * BNC Sensor(s):
 * - None
 * 
 * Note: the feather needs to be removed from the board when updated, 
 * since the bootloader does not set nKILL high (which will turn off the device after a few hundreds of ms).
 * 
 * 
 * Arduino IDE installation:
 * ========================
 *     Download and install the Arduino IDE (At least v1.8)
 *     Start the Arduino IDE
 *     Go into Preferences
 *     Add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json as an 'Additional Board Manager URL'
 *     Restart the Arduino IDE
 *     
 *     Open the Boards Manager option from the Tools -> Board menu and install 'Adafruit nRF52 by Adafruit'
 *     Once the BSP is installed (takes long time with a long delay in the middle), select
 *     Adafruit Bluefruit nRF52840 Feather Express (for the nRF52840) from the Tools -> Board menu
 *     
 * 
 * 
 * To update the bootloader:
 * =========================
 * Step 1:
 * Tools -> Board: Adafruit Feather nRF52840 Express
 * select "Bootloader DFU for Bluefruit nRF52" for Tools->Programmer
 * Select the right com port
 * Tools->Burn Bootloader 
 * 
 * Step 2:
 * Double-click the Reset button. NeoPixel turns green.
 * Drag the downloaded .uf2 file to FTHR840BOOT. The LED will flash.
 * 
 */

/* Set the wire library transmission hub */
#define DAC6571_ADDR 0x4C

/* Set the Hamamatsu sensor, false if none */
//String hamamatsusensor = "c12880ma";
String hamamatsusensor = "c14384ma-01";

int SPECTRO_CHANNELS; // Nr of spectral channels in sensor
int START_CHANNEL; // First channel with valid data
int END_CHANNEL; // Last channel with valid data

#define MAXIMUM_CHANNELS          288 // MMaximum Nr of spectral channels in avaliable sensors 
//#define MAXIMUM_CHANNELS          256 // MMaximum Nr of spectral channels in avaliable sensors

/* Set the RS485 MODBUS steel-pin sensors */
String rs485ModBusSensor = "npk-s";

#define BNC               A0 // The BNC (coaxial) port signal - mainly for Ion Selective Electrodes
#define SPECTRO_AN        A1 // The analog video signal for the spectrometer
#define GX16_AN           A2 // The aviator (GX) port analog sensor signal 
#define nKILL             17
#define nPBINT            18
#define ONEWIRE           19

#define GPIOSCK           26  // NC
#define nRE               25
#define GPIOMISO          24
#define GPIORX            1
#define GPIOTX            0
#define GPIOD2            2

#define SPECTRO_CLK       13 // Spectroemeter clock
#define SPECTRO_ST        12 // Spectrometer start
#define SPECTRO_TRG       11 // Spectroemter trigger
#define SPECTRO_EOS       10
#define SPECTRO_BOOST_EN  9 // Spectrometer booster engage - on/off needs to be turned off after scanning to allow sensor to recover

#define LED_BUCK_EN       6 // muzzle LED light buck engage - some light drains power and must be turned off when not in use
#define LED_PULSE         5 // LED light Pulse on/off

#define GPIOSCL           23
#define GPIOSDA           22

#define USERSWITCH        7
#define BATTERYV2         A6
#define NEOPIXEL          8

/* END pin definitions */

/* START Define the muzzle ONEWIRE EEPROM memory */

#include "OneWire.h"
OneWire ds(ONEWIRE);                    // OneWire bus on digital pin 19 (defiend as ONEWIRE aboe)
// Definde the 1-wire address @ 8 bytes
byte OneWireAddr[8]; // The 1-wire unique ID

/* END Define the muzzle ONEWIRE EEPROM memory */

#include <Adafruit_NeoPixel.h>    //  Library that provides NeoPixel functions

// -- Create a NeoPixel object called onePixel that addresses 1 pixel in pin 8
Adafruit_NeoPixel rgbPixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h> // for Serial
#endif

#include <Wire.h>
#include "SHT2x.h"

SHT2x sht;

/* START Modbus setup for communication with RS485+MODBUS sensors */
#include <ModbusMaster.h> 

void PreTransmission(){
  // Set transmit mode
  digitalWrite(nRE, HIGH);
}

void PostTransmission(){
  // Set receive mode
  digitalWrite(nRE, LOW);
}

// instantiate ModbusMaster object
ModbusMaster node;

/* END ModbusMaster For communication with RS485+MODBUS sensors */

void setup() {

  pinMode(BNC,      INPUT);  // BNc connection - usually an Ion Selecteve Electrode (ISE)
  pinMode(SPECTRO_AN,   INPUT);  // Spectro video analog signal
  pinMode(GX16_AN,      INPUT);  // Aalog signal from GX16 expansion socket)
  pinMode(nKILL,    OUTPUT); // #KILL
  pinMode(nPBINT,   INPUT);  // #PBINT (pushbutton)
  pinMode(ONEWIRE,   INPUT);  // 1W interface (pulled up to 3v3 by 2k2)
  pinMode(SCK,      OUTPUT); // - (not connected)
  pinMode(nRE,      OUTPUT); // #RE (Receive enabled, RS485)
  pinMode(GPIOMISO, OUTPUT); // - (not connected)
  pinMode(GPIORX,   INPUT);  // Rx (Uart)
  pinMode(GPIOTX,   OUTPUT); // Tx (Uart)
  pinMode(GPIOD2,   OUTPUT); // - (not connected)

  pinMode(SPECTRO_CLK,   OUTPUT); // Spectrometer clock pulse
  pinMode(SPECTRO_ST,    OUTPUT); // Spectrometer start pulse
  pinMode(SPECTRO_TRG,   INPUT);  // Spectrometer trigger pulse
  pinMode(SPECTRO_EOS,   INPUT);  // SPECTRO_EOS
  pinMode(SPECTRO_BOOST_EN, OUTPUT); // Spectrometer end of scan
  
  pinMode(LED_BUCK_EN,  OUTPUT); // LED_BUCK_EN
  pinMode(LED_PULSE,    OUTPUT); // LED_PULSE
  
  pinMode(GPIOSCL,  INPUT);  // SCL
  pinMode(GPIOSDA,  INPUT);  // SDA

  pinMode(USERSWITCH,   INPUT);  // User switch

  pinMode(A6,           INPUT);  // Battery voltage/2
  pinMode(LED_BUILTIN,  OUTPUT); // Red LED
  pinMode(NEOPIXEL,     OUTPUT); // RGB LED

  digitalWrite(nKILL, HIGH);     // Keep the unit on. 3v3 feeds Rh/T sensor, Ph sensor and LED driver DAC
  digitalWrite(nRE, HIGH);       // Set transmit mode
  digitalWrite(SPECTRO_CLK, LOW);
  digitalWrite(SPECTRO_ST, LOW);

  digitalWrite(SPECTRO_BOOST_EN, LOW);

  digitalWrite(LED_BUCK_EN, LOW);

  Serial.begin(115200);

  // Light source
  // Turn off LED_PULSE before starting
  //  digitalWrite(LED_PULSE, LOW);
  digitalWrite(LED_PULSE, LOW);

  if ( (rs485ModBusSensor == "npkphcth-s") || (rs485ModBusSensor == "npk-s") || (rs485ModBusSensor == "ph-s") ) {
    Serial1.begin(4800);
  } else {
    Serial1.begin(9600);
  }
  
  node.begin(1, Serial1);
  node.preTransmission(PreTransmission);
  node.postTransmission(PostTransmission);
  
  sht.begin();

  // Prepare the builtin neopixel - will loop the rainbow to indicate that spetrometer is scanning
  rgbPixel.begin();             // Start the NeoPixel object
  rgbPixel.clear();             // Set NeoPixel color to black (0,0,0)
  rgbPixel.setBrightness(20);   // Affects all subsequent settings
  rgbPixel.show();              // Update the pixel state
  
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  delay(100);   
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);

  /* START Define the channels of the main spectrometer */
    
  if (hamamatsusensor == "c12880ma") {
    SPECTRO_CHANNELS=288; // Nr of spectral channels in sensor
    START_CHANNEL=0; // First channel with valid data
    END_CHANNEL=288; // Last channel with valid data
  } else if (hamamatsusensor == "c14384ma-01") {
    SPECTRO_CHANNELS=256; // Nr of spectral channels in sensor
    START_CHANNEL=64; // First channel with valid data
    END_CHANNEL=256; // Last channel with valid data
  } else {
    SPECTRO_CHANNELS=0; // Nr of spectral channels in sensor
    START_CHANNEL=0; // First channel with valid data
    END_CHANNEL=0; // Last channel with valid data
  }
  
}

uint16_t spectra[MAXIMUM_CHANNELS];
uint16_t spectraSignal[MAXIMUM_CHANNELS];
uint16_t spectraDark[MAXIMUM_CHANNELS];

bool diagnos = false;

// Set DAC voltage (raw DAC value)
// 12 bit, 0-4095 <=> 0-3.3V
void DAC6571_setVoltage(uint16_t v)
{ 
  uint16_t vh,vl;
 
  vh=(v>>8) & 0xff;
  vl=v & 0xff;
  
  Wire.beginTransmission(DAC6571_ADDR);
  Wire.write(vh); // PD bits =0 (bit 4-5) = Normal operation
  Wire.write(vl);
  Wire.endTransmission();
}

#define RA 68.0
#define RB 16.9
#define RD 75.0
#define DAC_VREF_mV 3300.0
#define DAC_MAX 4095.0

// Set the LED PULSE milli Volt

void LED_setVoltage_mV(float vset_mV)
{
  float vdac_mV=600*(RD/RB+1)+(600-vset_mV)*RD/RA;
//  (600.0*(1+RA/RB)-vset_mV)*RD/RA+600.0;
  float r=vdac_mV/DAC_VREF_mV*DAC_MAX;

  DAC6571_setVoltage((uint16_t)r);
}

/* END LED voltage control */
  
/* START Search for the connected muzzle 1-wire address and check that it is OK, return false or true */

bool SearchOneWireAddress(byte* address) //Search for address and confirm it
{
  int i;
  if (diagnos){
    Serial.println("SearchOneWireAddress:");
  }
  ds.reset_search();
  
  if ( !ds.search(OneWireAddr))
  {
    // no ONEWIRE ATTACH - red blink
    rgbPixel.setPixelColor(0, 100, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    if (diagnos) {
      Serial.print("  No muzzle (eeprom device) found.\n");
    } 
    ds.reset_search();
    delay(250);
    rgbPixel.setPixelColor(0, 0, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    return false;
  }

  if (diagnos) {
    // Print the 1wire ID
    Serial.print("  muzzle HEX ID=");
    for( i = 0; i < 8; i++) {
      Serial.print(OneWireAddr[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");  
  }
  
  // Cyclic Redundance Check (CRC)
  if ( OneWire::crc8( OneWireAddr, 7) != OneWireAddr[7]) {
    // CRC error - orange blink
    rgbPixel.setPixelColor(50, 50, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    if (diagnos) {
      Serial.print("  Invalid muzzle (eeprom device) address!\n");
    }
    delay(250);
    rgbPixel.setPixelColor(100, 100, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    return false;
  }
  
  if ( OneWireAddr[0] != 0x2D) {
    // wrong ONEWIRE ATTACH - yellow blink
    rgbPixel.setPixelColor(100, 100, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    if (diagnos) {
      Serial.print("  Device is not a 1-wire Eeprom.\n");
    } 
    delay(250);
    rgbPixel.setPixelColor(0, 0, 0, 0); //  Set pixel 0 to (r,g,b) color value
    rgbPixel.show(); //  Update pixel state
    return false;
  }

  return true;
}

/* END Search for the connected muzzle 1-wire address and check that it is OK*/

/* START Read the complete content of the 1-wide DS2431 memory and
 *  retrieve the values of the predefined parameters */

void Read1WDS2431Mem(String* eepromStrings, int* eepromInts, bool diagnos)
{
  int i;
  ds.reset();
  ds.select(OneWireAddr);
  ds.write(0xF0,1);  // Read Memory
  ds.write(0x00,1);  //Read Offset 0000h
  ds.write(0x00,1);

  String muzzleid;
  String emittor1;
  String emittor1minV;
  String emittor1maxV;
  String emittor1resistor;
  String emittor2;
  String emittor2minV;
  String emittor2maxV;
  String emittor2resistor;
  String forwardV;
  String integrationtime;
  String stabilisationtime;
  String creationdate;
 
  for ( i = 0; i < 120; i++) //whole mem is 144 
  {
    char c = ds.read();
    
    if (i < 8) {
      muzzleid += c;
    } else if (i < 24) {
      emittor1 += c;  
    } else if (i < 32) {
      emittor1minV += c;  
    } else if (i < 40) {
      emittor1maxV += c;
    } else if (i < 48) {
      emittor1resistor += c; 
    } else if (i < 64) {
      emittor2 += c;  
    } else if (i < 72) {
      emittor2minV += c; 
    } else if (i < 80) {
      emittor2maxV += c;
    } else if (i < 88) {
      emittor2resistor += c;
    } else if (i < 96) {
      forwardV += c;
    } else if (i < 104) {
      integrationtime += c;
    } else if (i < 112) {
      stabilisationtime += c;
    } else if (i < 120) {
      creationdate += c;
    } 
  }

  // Convert strings to string array

  eepromStrings[0] = muzzleid;
  eepromStrings[1] = emittor1;
  eepromStrings[2] = emittor2;
  eepromStrings[3] = creationdate;
 
  // Convert integers to integer array
  eepromInts[0] = emittor1minV.toInt();
  eepromInts[1] = emittor1maxV.toInt();
  eepromInts[2] = emittor1resistor.toInt();
  eepromInts[3] = emittor2minV.toInt();
  eepromInts[4] = emittor2maxV.toInt();
  eepromInts[5] = emittor2resistor.toInt();
  eepromInts[6] = forwardV.toInt();
  eepromInts[7] = integrationtime.toInt();
  eepromInts[8] = stabilisationtime.toInt();

  if (diagnos) {
    Serial.println("  muzzleid:" + eepromStrings[0]);

    Serial.println("  emittor1:" + eepromStrings[1]);

    Serial.print("  emittor1minV:");
    Serial.println(eepromInts[0]);
 
    Serial.print("  emittor1maxV:");
    Serial.println(eepromInts[1]);

    Serial.print("  emittor1resistor:");
    Serial.println(eepromInts[2]);
    
    Serial.println("  emittor2:" + eepromStrings[2]);

    Serial.print("  emittor2minV:");
    Serial.println(eepromInts[3]);
 
    Serial.print("  emittor2maxV:");
    Serial.println(eepromInts[4]);
 
    Serial.print("  emittor2resistor:");
    Serial.println(eepromInts[5]);

    Serial.print("  forwardV:");
    Serial.println(eepromInts[6]);

    Serial.print("  integrationtime:");
    Serial.println(eepromInts[7]);

    Serial.print("  stabilisationtime:");
    Serial.println(eepromInts[8]);

    Serial.println("  creationdate:" + eepromStrings[3]);

    Serial.println();   
  }
  
}

/* END Read the complete content of the muzzle 1-wide DS2431 memory */

/* START main spectra scan */

void readSpectrometer(){

  int delayTime = 25; // delay time in microseconds

  // Start clock cycle and set start pulse to signal start
  digitalWrite(SPECTRO_CLK, LOW);
  delayMicroseconds(delayTime);
  digitalWrite(SPECTRO_CLK, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(SPECTRO_CLK, LOW);
  digitalWrite(SPECTRO_ST, HIGH);
  delayMicroseconds(delayTime);

  //Sample for a period of time
  for(int i = 0; i < 15; i++){

      digitalWrite(SPECTRO_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPECTRO_CLK, LOW);
      delayMicroseconds(delayTime); 
 
  }

  //Set SPECTRO_ST to low
  digitalWrite(SPECTRO_ST, LOW);

  //Sample for a period of time
  for(int i = 0; i < 85; i++){

      digitalWrite(SPECTRO_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPECTRO_CLK, LOW);
      delayMicroseconds(delayTime); 
      
  }

  //One more clock pulse before the actual read
  digitalWrite(SPECTRO_CLK, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(SPECTRO_CLK, LOW);
  delayMicroseconds(delayTime);

  //Read from SPECTRO_VIDEO
  for(int i = 0; i < SPECTRO_CHANNELS; i++){

      spectra[i] = analogRead(SPECTRO_AN);
      
      digitalWrite(SPECTRO_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPECTRO_CLK, LOW);
      delayMicroseconds(delayTime);
        
  }

  //Set SPECTRO_ST to high
  digitalWrite(SPECTRO_ST, HIGH);

  //Sample for a small amount of time
  for(int i = 0; i < 7; i++){
    
      digitalWrite(SPECTRO_CLK, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(SPECTRO_CLK, LOW);
      delayMicroseconds(delayTime);
    
  }

  digitalWrite(SPECTRO_CLK, HIGH);
  delayMicroseconds(delayTime);
  
}

// print results from main spectral sensor

void value_print_Main_spectra_dark(String sensor, int start_ch, int end_ch, int blanks){
  int i;
  int n = 0;


  while (n < 2) { 
    if (diagnos){
      Serial.print("      data=");
    } else {
      Serial.print("data=");
    }

    Serial.print(sensor);
    Serial.print(':');
    Serial.print(n);
    Serial.print(',');
    for ( i = start_ch ; i < end_ch ; i++ )
    {
      if (n == 0) {
        Serial.print(spectraDark[i]);     
      } else {
        Serial.print(spectraSignal[i]);
      }
      
      if (i < end_ch-1){
        Serial.print(',');
      }    
    }
    Serial.print("\n");
    n += 1;
  }
}

void value_print_Main_raw_scan(String sensor, int start_ch, int end_ch){
  int i;
  int n = 0;
  while (n < 2) { 
    Serial.print("raw=");
    Serial.print(sensor);
    Serial.print(':');

    for ( i = start_ch ; i < end_ch ; i++ )
    {

        Serial.print(spectra[i]);     

      if (i < end_ch-1){
        Serial.print(',');
      }    
    }
    Serial.print("\n");
    n += 1;
  }
}

void value_print_SortedSpectra(String sensor, uint16_t sortedSpectra[], int start_ch, int end_ch){

  Serial.print("  sortedSpectra=");
  Serial.print(sensor);
  Serial.print(':');

  for ( int i = start_ch ; i < end_ch ; i++ )
  {

    Serial.print(sortedSpectra[i]);     

    if (i < end_ch-1){
      Serial.print(',');
    }    
  }
    Serial.print("\n");

}

// Function to copy 'len' elements from 'src' to 'dst'
void copy(uint16_t* src, uint16_t* dst, int startPixel, int endPixel) {
  if (diagnos){
    Serial.print("        Copying pixel: ");
    Serial.print(startPixel);
    Serial.print(" to ");
    Serial.println(endPixel);
  }
    
  for (int i = startPixel; i < endPixel; i++) {
    dst[i] = src[i];
  }
}

void hamamatsu_spectra_scan(String sensor, int SPECTRO_stabilisation_time, int vset_mV, int nScans, int n_integration_times,
                            int SPECTRO_integration_times[],int SPECTRO_start_pixels[], int SPECTRO_end_pixels[], int blanks=0){
  
  int integrationTime;
  int startPixel;
  int endPixel;

  const int N = n_integration_times;
  
  float LED_vset_mV = vset_mV;

  // reset spectraScan and darkScan
  memset(spectraSignal, 0, sizeof(spectraSignal));
  memset(spectraDark, 0, sizeof(spectraDark));
  
  int red = 0; int green = 0; int blue = 0;
  uint8_t scannr = 0;
  int loopnr = 0;

  while (scannr < nScans) { 
    if (diagnos) {
      Serial.print("    scannr: ");
      Serial.println(scannr);
    }
    
    
    for (int j = N-1; j>=0; j--){

      if (diagnos) {  
        Serial.print("      for loop (integration time):");
        Serial.println(j);
      }
      
      loopnr += 1;

      integrationTime = SPECTRO_integration_times[j];
      startPixel = SPECTRO_start_pixels[j];
      endPixel = SPECTRO_end_pixels[j];

      // Step rgbpixel values blue - gree - yellow - red - blue
      if ( (loopnr % 4) == 0) { red=0; green=0; blue=100; } 
      else if ( (loopnr % 3) == 0) { red=100; green=0; blue=0; }
      else if ( (loopnr % 2) == 0) { red=100; green=100; blue=0; }
      else  { red=0; green=100; blue=0; }
    
      rgbPixel.setPixelColor(0, red, green, blue); //  Set pixel 0 to (r,g,b) color value
      rgbPixel.show(); //  Update pixel state
  
      // Turn on light source
      LED_setVoltage_mV(LED_vset_mV);
  
      digitalWrite(LED_PULSE, HIGH);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      
      // Let light stabilize
      delay (SPECTRO_stabilisation_time);
      
      // Empty spectrometer wells to get clean read 
      readSpectrometer();
  
      // Fill up spectrometer wells
      delay (integrationTime);
      
      // Read the sample signal
      readSpectrometer();
      
      // copy the spectra to spectraSignal
      copy(spectra, spectraSignal, startPixel, endPixel); //Call a function copy 
  
      // Turn off light source
      LED_setVoltage_mV(0);
    
      digitalWrite(LED_PULSE, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
      
      // Wait a bit for light to go off completely
      delay (SPECTRO_stabilisation_time);
      
      // Empty spectrometer wells to get clean read 
      readSpectrometer();
  
      // Set same integration time as in sample read
      delay (integrationTime);
      
      // Read the dark signal
      readSpectrometer();
      
      // copy the spectra to darkSignal
      copy(spectra, spectraDark, startPixel, endPixel); //Call a function copy
      
    } // end for 
    
    // Write out the scan result
    value_print_Main_spectra_dark(sensor, START_CHANNEL, END_CHANNEL, blanks);
    
    scannr += 1;
    
  } // end while
  
  rgbPixel.setPixelColor(0, 0, 0, 0); //  Turn off neopixel
  rgbPixel.show(); //  Update pixel state
}

/* END main spectra scan */


int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  uint16_t  a = *((uint16_t  *)cmp1);
  uint16_t  b = *((uint16_t  *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

void SetIntegrationTimes(String sensor, int SPECTRO_stabilisation_time, int vset_mV, int n_integration_times, int maxDN,
                        int iniIntegrationTime, int ascendingIntegrationTimeStep, int descedingIntegrationTimeStep,
                        int SPECTRO_integration_times[], int SPECTRO_start_pixels[], int SPECTRO_end_pixels[]){
  
  // Set integration time from single scan
  int nscans = 1;
  // define the arrays (size = 1) to send to hamamatsu_spectra_scan
  int startPixels[1];
  int endPixels[1];
  int integrationTimes[1];

  // Set integrationtime and define the full range of channels/pixels
  integrationTimes[0] = iniIntegrationTime;
  startPixels[0] = START_CHANNEL;
  endPixels[0] = END_CHANNEL;
  
  // The boolean variables ascending and cont are used for detrmining if the intergration should increase or decrease
  bool ascending = true;
  bool cont = true;

  int minSignal = 10;
  int maxSignal = 10;

  if (diagnos) {
    Serial.print("  n_integration_times: ");
    Serial.println( n_integration_times);
    Serial.print("  start pixel: ");
    Serial.println(startPixels[0]);
    Serial.print("  end pixel: ");
    Serial.println(endPixels[0]);  
  }
  
  while (cont) {
    // reset spectra
    memset(spectra, 0, sizeof(spectra));
    
    if (diagnos) {
      Serial.print("  integration time: ");
      Serial.println(integrationTimes[0]);   
    }
    
    // Wait a bit longer with the spectrometer and LED in off state
    delay(500);

    // Power up spectral sensor
    digitalWrite(SPECTRO_BOOST_EN, HIGH);
    // Start LED driver 
    digitalWrite(LED_BUCK_EN, HIGH);

    // Do the sepctral scan
    hamamatsu_spectra_scan(sensor, SPECTRO_stabilisation_time, vset_mV, nscans, 1,
                            integrationTimes, startPixels, endPixels);

    // Turn off LED driver 
    digitalWrite(LED_BUCK_EN, LOW);
    // Power down spectral sensor
    digitalWrite(SPECTRO_BOOST_EN, LOW);

    // Get min and max signal for the scan
    minSignal = spectra[0]; 
    maxSignal = spectra[0];
    
    for (int i = START_CHANNEL; i < END_CHANNEL ; i++) {
      if (spectraSignal[i] > maxSignal) {
         maxSignal = spectraSignal[i];
      }
      if (spectra[i] < minSignal) {
         minSignal = spectraSignal[i];
      }
    }
    
    if (diagnos) {
      Serial.print("      maxSignal: ");
      Serial.println(maxSignal);
      Serial.print("      minSignal: ");
      Serial.println(minSignal); 
    }
    
    if (ascending && maxSignal < maxDN) {

      integrationTimes[0] += ascendingIntegrationTimeStep;
      
    } else {
      ascending = false;
      integrationTimes[0] -= descedingIntegrationTimeStep;
      if (maxSignal < maxDN){
        cont = false;
      }
      
    } 

  }

  // sort the last scan in place
  int scan_length = sizeof(spectraSignal) / sizeof(spectraSignal[0]);
  qsort(spectraSignal, scan_length, sizeof(spectraSignal[0]), sort_desc);

  // extract the channels with valid values
  int LIVE_CHANNELS = END_CHANNEL-START_CHANNEL;
  uint16_t sortedSpectra[LIVE_CHANNELS];
  for (int s = 0; s <= LIVE_CHANNELS; s++) {
      sortedSpectra[s] = spectraSignal[s];
  }

  if (diagnos) {
    Serial.print("\n  Tuned integration time: ");
    Serial.println(integrationTimes[0]);
    // print the sorted spectra
    value_print_SortedSpectra(sensor, sortedSpectra, 0, LIVE_CHANNELS);
  }
    
  int SpectraSplit[n_integration_times];
  int integrationTimeFactors[n_integration_times];
  int split = 0;
  int ns = 0;
  int intTime = integrationTimes[0];
  

  // Split with linear division
  // Most straight forward and OK result
  split = maxSignal/n_integration_times;
  for (int r = 0; r < n_integration_times-1; r++) {
    ns = n_integration_times-1-r;
    SpectraSplit[r] = split*(ns);
    intTime = integrationTimes[0]*(r+1);
    integrationTimeFactors[r] = intTime;   
  }
  
  /*
  // Sequential split in hals; gives a wider center but is generally not better
  split = maxSignal;
  for (int r = 0; r < n_integration_times-1; r++) {
    split /= 2;
    SpectraSplit[r] = split;
    intTime *= 2;
    integrationTimeFactors[r] = intTime
  }
  */
  
  // reset spectraScan and darkScan
  memset(spectraSignal, 0, sizeof(spectraSignal));
  memset(spectraDark, 0, sizeof(spectraDark));
  
  // rescan as the previous scan is sorted
  if (diagnos) {
    Serial.print("  Rescanning tuned integartion time: ");
    Serial.println(integrationTimes[0]);
  }
  // Wait a bit with the spectrometer and LED in off state
  delay(400);
  // Power up spectrometer
  digitalWrite(SPECTRO_BOOST_EN, HIGH);
  // Start LED driver 
  digitalWrite(LED_BUCK_EN, HIGH);
    
  hamamatsu_spectra_scan(sensor, SPECTRO_stabilisation_time, vset_mV, nscans, 1,
                            integrationTimes, startPixels, endPixels);

  // Turn off LED driver 
  digitalWrite(LED_BUCK_EN, LOW);
  // Power down spectrometer
  digitalWrite(SPECTRO_BOOST_EN, LOW);


  if (diagnos) {
    Serial.print("  Splitting the tuned spectra histogram: ");
  }
   
  // Divide the spectra using the split levels
  int s;
  int n;
  for (int r = 0; r < n_integration_times-1; r++) {
    split = SpectraSplit[r];
    Serial.print("    split@: ");
    Serial.println(SpectraSplit[r]);
    s = 0;
    n = 0;
 
    SPECTRO_integration_times[r] = integrationTimeFactors[r];
    while (s < split){
      s = spectraSignal[n];
      SPECTRO_start_pixels[r] = n;
      n+=1;
    }
    n = END_CHANNEL;
    s = 0;
    while (s < split){
      s = spectraSignal[n];
      SPECTRO_end_pixels[r] = n;
      n-=1;
    }
    
  }
  
  // Set the last integrationTime, startPixel and endPixel
  SPECTRO_integration_times[n_integration_times-1] = integrationTimes[0]*(n_integration_times);
  SPECTRO_start_pixels[n_integration_times-1] = START_CHANNEL;
  SPECTRO_end_pixels[n_integration_times-1] = END_CHANNEL;
  if (diagnos){
     Serial.println("Finished automatic setting of integrationtimes");
   }
}

/* END SetIntegraationTime */

/* START receive commands */

const byte numChars = 196;

char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/* END receive commands */

char userInput;
String inString = "";

void loop()
{
  
  bool fullprint = 1;
  bool valueprint = 0;
  int x;
  int adcvalue = 0;
  String metadata;
  float vbat;

  // Set conversion between 10-bit ADC and input range
  float mv_per_lsb = 3600.0F/1024.0F; // 10-bit ADC with 3.6V input range
  
  delay (200);
  digitalWrite(LED_RED, HIGH);

  // Listen for input string
  recvWithStartEndMarkers();
  
  if (newData == true) {
    inString = String(receivedChars);
    
    // split the incoming string into parameters, 1=sensor; 2=nScans, 3 and more = sensor specific parameters
    String sensor = getValue(inString, ':', 0);
    String nscanStr = getValue(inString, ':', 1);
    String diagnosStr = getValue(inString, ':', 2);
    int nscans = nscanStr.toInt();
    diagnos = diagnosStr.toInt();

    // Create a string array to hold the extra parameters
    
    String xparam[18];
    // Set the xparams
    for(x=0; x<20; x++) {
      xparam[x]= getValue(inString, ':', x+3);    
    }

    if (diagnos) {

      // Print out complete inString
      Serial.println("Diagnostic mode");
      Serial.println("inString: " + inString);
      
      // Print out all input parameters
      Serial.println("Parameters (starting)");
      Serial.println("  sensor: " + sensor);
      Serial.println("  scans: " + nscanStr);
      Serial.println("  diagnose: " + diagnosStr);
      for(x=0; x<20; x++) {
        Serial.print("  xparam");  
        Serial.print (x);
        Serial.println(":" +  xparam[x]);
      }
      Serial.println("Parameters (finished)");

      // Get the battery status - if battery is below vset_mV the spectral scan is not done
      adcvalue = analogRead(PIN_VBAT);
      vbat=(float)adcvalue * mv_per_lsb *2;

      Serial.print("\nBattery status (VBAT): ");
      Serial.print(vbat);
      Serial.println(" mV");

    }

    // Identify the sensor asked for

    if ((sensor == "c12880ma") || (sensor == "c14384ma-01")) { 
      // example code diagnostic run set integrationtimes: <c14384ma-01:2:1:1206VINI:800:2500:200:0:2:50:100:200:200:64:256>
      // example code diagnostic run auto find integrationtimes: <c14384ma-01:2:1:1206VINI:800:2500:200:1:3:200:20:5>

      /*
      c14384ma-01:6:1:
      xp0 = muzzle = 1206VINI:
      xp1 = maxDN = 800:
      xp2 = vset_mV = 1500:
      xp3 = stabilisation_time = 200;

      xp4 = autoIntegrationTime = 1:
      xp5 = nIntegrationTimes = 2:

      // From hereon the params are only needed if autoIntegrationTime == 0
      xp6 = integrationsTime0 = 200:
      xp7 = startPixelIntegrationTime0 = 89:
      xp8 = endPixelIntegrationTime0 = 200:

      xp9 = integrationsTime1 = 200:
      xp10 = startPixelIntegrationTime1 = 89:
      xp11 = endPixelIntegrationTime1 = 200:

      xp12 = integrationsTime2 = 200:
      xp13 = startPixelIntegrationTime2 = 89:
      xp14 = endPixelIntegrationTime2 = 200:

      xp15 = integrationsTime3 = 200:
      xp16 = startPixelIntegrationTime3 = 89:
      xp17 = endPixelIntegrationTime3 = 200:    
      */

      // Set the initial parameters
      int maxDN = xparam[1].toInt();
      int vset_mV = xparam[2].toInt();
      int SPECTRO_stabilisation_time = xparam[3].toInt();
      int autoIntegrationTime = xparam[4].toInt();
      int n_integration_times = xparam[5].toInt();
      int SPECTRO_integration_times[5];
      int SPECTRO_start_pixels[5];
      int SPECTRO_end_pixels[5];
      // Set parameters derived from eeprom
      String muzzleId;
      String emittor1;
      String emittor2;
      

      if (diagnos) {
        Serial.println();
      }
        
      // Search for and identify the attached muzzle via its 1-wire EEPROM memory
      bool OneWireOK = SearchOneWireAddress(OneWireAddr);

      if (diagnos){
        Serial.print("  OneWireOK: ");
        Serial.println(OneWireOK);  
      }

      if (OneWireOK) {
        int m;
        // Set the arrays that hold the data from the muzzle EEPROM
        int eepromInts[9];
        String eepromStrings[4];

        Read1WDS2431Mem(eepromStrings, eepromInts, diagnos);

        eepromStrings[0].trim();
        eepromStrings[1].trim();
        eepromStrings[2].trim();

        muzzleId = eepromStrings[0];
        emittor1 = eepromStrings[1];
        emittor2 = eepromStrings[2];
        
        if ( xparam[0] != eepromStrings[0] ){
          if (diagnos) {
            Serial.println("WARNING - the muzzle id (" + eepromStrings[0] + ") is different compared to the command sent: " + xparam[0]);
          }
        }

        if (xparam[2].toInt() <= 0){
          vset_mV = eepromInts[6];
        } else {
          vset_mV = xparam[2].toInt();
          if (vset_mV > eepromInts[1] & diagnos) {              
            Serial.print("WARNING - the user defined LED power, " +  xparam[2] + " is higher than the maximum allowed for the muzzle primary emittor: ");
            Serial.println(eepromInts[1]);   
          } else if (eepromInts[4] > 0 & vset_mV > eepromInts[5] & diagnos) {
            Serial.print("WARNING - the user defined LED power, " + xparam[1] + " is higher than the maximum allowed for the muzzle secondary emittor: ");
            Serial.println(eepromInts[5]);
          } else if (vset_mV < eepromInts[0]  & diagnos) {
            Serial.print("WARNING - the user defined LED power, " + xparam[1] + " is lower than the minimum threshold for the muzzle primary emittor: ");
            Serial.println(eepromStrings[0]);
          } else if (eepromInts[2] > 0 & vset_mV < eepromInts[4]  & diagnos) {
            Serial.print("WARNING - the user defined LED power, " + xparam[1] + "  is lower than the minimum threshold for the muzzle secondary emittor: ");
            Serial.println(eepromStrings[4]);
          }
        }
        
        if (vset_mV > vbat) {
            Serial.print("WARNING - the available battery voltage, " + xparam[1] + "  is lower than the required voltage: ");
            Serial.println("EXITING");
        }
        
        if (xparam[3].toInt() <= 0){
          SPECTRO_stabilisation_time = eepromInts[6];
        } 
        
      } else {
        // no onewire EEPROM

        muzzleId = "unknown";
        emittor1 = "unknown";
        emittor2 = "unknown";
        
      }

      if (autoIntegrationTime){

        int iniIntegrationTime=xparam[6].toInt();
        int ascendingIntegrationTimeStep=xparam[7].toInt();
        int descedingIntegrationTimeStep=xparam[8].toInt();
        if (diagnos){
          Serial.println("\nAutomatic setting of integrationtimes");
        }
        
        SetIntegrationTimes(sensor, SPECTRO_stabilisation_time, vset_mV, n_integration_times, maxDN, iniIntegrationTime,
                            ascendingIntegrationTimeStep,descedingIntegrationTimeStep,
                            SPECTRO_integration_times, SPECTRO_start_pixels, SPECTRO_end_pixels);
      } else {

        for(int t = 0; t < n_integration_times; t++){
        int u = 6+t*3;
          SPECTRO_integration_times[t] = xparam[u].toInt();
          SPECTRO_start_pixels[t] = xparam[u+1].toInt();
          SPECTRO_end_pixels[t] = xparam[u+2].toInt();
          if (diagnos){
            Serial.print("IntegrationTime");
            Serial.print(t);
            Serial.print(":");
            Serial.print( xparam[u] );
            Serial.print(" startPixel:");
            Serial.print( xparam[u+1] );
            Serial.print(" endPixel:");
            Serial.println( xparam[u+2] );
          }  
        }
      }
 
      // Get the battery status - if battery is below vset_mV the spectral scan is not done
      adcvalue = analogRead(PIN_VBAT);
      vbat=(float)adcvalue * mv_per_lsb *2;

      // Wait a bit with the spectrometer and LED in off state
      delay(200);
      digitalWrite(SPECTRO_BOOST_EN, HIGH);
      // Start LED driver 
      digitalWrite(LED_BUCK_EN, HIGH);

      Serial.println("\nStart of spectra scan\n");
      if (diagnos) {
        Serial.print("  sensor: ");
        Serial.println(sensor);
        Serial.print("  SPECTRO_stabilisation_time: ");
        Serial.println(SPECTRO_stabilisation_time);
        Serial.print("  vset_mV: ");
        Serial.println(vset_mV);
        for(int t = 0; t < n_integration_times; t++){
          Serial.print("  IntegrationTime");
          Serial.print(t);
          Serial.print(":");
          Serial.print( SPECTRO_integration_times[t] );
          Serial.print(" startPixel:");
          Serial.print( SPECTRO_start_pixels[t] );
          Serial.print(" endPixel:");
          Serial.println( SPECTRO_end_pixels[t] );
        }
      }

      // Create a json object of the metadata for the spectral emittor(s) and their settings
      metadata = "{\"muzzleid\":\"" + muzzleId + "\",\"emittor1\":\"" + emittor1 + "\",\"emittor2\":\"" + emittor2 + "\""; 
      metadata.concat(",\"vset_mV\":");
      metadata.concat( vset_mV );
      metadata.concat(",\"stabilisationtime\":");
      metadata.concat(SPECTRO_stabilisation_time);
        
      metadata.concat(",\"integrationtimes\": {");
        
      for(int t = 0; t < n_integration_times; t++){
        metadata.concat("\"IntegrationTime");
        metadata.concat(t);
        metadata.concat("\":{\"ms\":");
     
        metadata.concat( SPECTRO_integration_times[t] );
        metadata.concat(",\"startPixel\":");
        metadata.concat( SPECTRO_start_pixels[t] );
        metadata.concat(",\"endPixel\":");
        metadata.concat( SPECTRO_end_pixels[t] );
        if (t == n_integration_times-1){
          metadata.concat( "}}");
        } else {
          metadata.concat( "},");
        }
        
          
      }
      metadata.concat( "}");

      hamamatsu_spectra_scan(sensor, SPECTRO_stabilisation_time, vset_mV, nscans, n_integration_times,
                            SPECTRO_integration_times, SPECTRO_start_pixels, SPECTRO_end_pixels);
                            
      // Turn off LED driver 
      digitalWrite(LED_BUCK_EN, LOW);
      digitalWrite(SPECTRO_BOOST_EN, LOW);
       
    } else {
      // No sensor detected
      Serial.println("ERROR: no sensor detector with inString:" + inString);
      metadata = "{}";
    }
    
    Serial.println("End of spectra scan");
    
    // print metadata as single row of data

    Serial.print( "metadata=" );
    Serial.println( metadata );

    newData = false;

    inString = "-99";
        
   }
    
 delay (200);
 digitalWrite(LED_RED, LOW);

}
