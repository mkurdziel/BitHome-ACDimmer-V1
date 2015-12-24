/*
  AC Light Control
  
  The hardware consists of an Triac to act as an A/C switch and
  an opto-isolator to give us a zero-crossing reference.
  The software uses two interrupts to control dimming of the light.
  The first is a hardware interrupt to detect the zero-cross of
  the AC sine wave, the second is software based and always running
  at 1/128 of the AC wave speed. After the zero-cross is detected
  the function check to make sure the proper dimming level has been
  reached and the light is turned on mid-wave, only providing
  partial current and therefore dimming our AC load.
  Modified to use the AC line frequency (half-period) as a reference point
  and fire the triacs based on that plus a count of dimming steps.
  Tracks the line frequency and adjusts accordingly.  Can set up to
  an estimated 512 steps of dimmer resolution.
  Apply an adjustment to account for the shape of the sinewave - giving linear power adjustment
  Curve from:
  time=arccos(power-1)/Pi
  where t is delay between 0 and 1 and a power of 2 is 100%
  
  The sense of the input to the dimming is also reversed from the previous version by the power mapping - as the analogue input is controlling
  the power out - not the delay time. 1024 in from an analogue input will set maximum power.
*/

#include <TimerOne.h>
#include <EEPROM.h>
#include <SyNetXBee.h>
#include <SyNetProtocol.h>
// Defines
#define PIN_LED     4
#define PIN_RESET   2
#define PIN_POT1 	  18	
#define PIN_POT2		19

// INFO SET API
#define INFO_API_ID       0x01
#define INFO_API_MANUFAC  0x02
#define INFO_API_PROFILE  0x03
#define INFO_API_REVISION 0x04
#define INFO_API_REMOTE   0x0F

#define EEPROM_ID         0
#define EEPROM_MANUFAC    2
#define EEPROM_PROFILE    4
#define EEPROM_REVISION   6
#define EEPROM_REMOTE     8

SyNetXBee xbee(Serial);

// Global functions
int byteIn;

two_byte_union m_SyNetUniqueID;
two_byte_union m_ManufacturerID;
two_byte_union m_ProfileID;
two_byte_union m_RevisionID;

int m_ledCount = 0;
byte m_prevPotIndex = 0;
byte m_potValues[4] = {0,2,3,1};
int m_potDir = 0;
// Member variables

// General
unsigned long int ZeroXTime[4] = {0,0,0,0};            // Timestamp in micros() of the zero crossing interrupts
unsigned long int DimStep;                             // How many micros() in each step of dimming
unsigned long int AvgPeriod;                           // The average line voltage period in micros()
unsigned long int PeriodResync = 3000;                 // Number of milliseconds between line freq measurements
unsigned long int ResetPeriod = PeriodResync;          // The timestamp in millis() when we will measure the period again
unsigned long int DimRes = 256;                        // How many steps of dimmer resolution
volatile unsigned long int DimStepCounter;             // For counting Timer1 interrupts
volatile unsigned long int FireTriac[4] = {255,0,0,0};   // When it's OK to fire the triacs, in counts of DimRes
volatile boolean zero_cross = 0;                       // Tels us we've crossed the zero line
byte TriacPin[1] = {5};                          // Which digital IO pins to use
byte m_powerStep = 0;
byte m_powerInc = 10;
byte PowerMap[256] = {
 255,245,241,237,235,232,230,228,226,224,223,221,220,218,217,215,214,213,211,210,209,208,207,205,204,203,202,201,200,199,198,197,
 196,195,194,193,192,192,191,190,189,188,187,186,185,185,184,183,182,181,181,180,179,178,177,177,176,175,174,174,173,172,171,171,
 170,169,168,168,167,166,165,165,164,163,163,162,161,161,160,159,158,158,157,156,156,155,154,154,153,152,152,151,150,150,149,148,
 148,147,146,146,145,144,144,143,143,142,141,141,140,139,139,138,137,137,136,135,135,134,134,133,132,132,131,130,130,129,128,128,
 127,127,126,125,125,124,123,123,122,121,121,120,120,119,118,118,117,116,116,115,114,114,113,112,112,111,111,110,109,109,108,107,
 107,106,105,105,104,103,103,102,101,101,100,99,99,98,97,97,96,95,94,94,93,92,92,91,90,90,89,88,87,87,86,85,84,84,83,82,81,81,80,
 79,78,78,77,76,75,74,74,73,72,71,70,70,69,68,67,66,65,64,63,63,62,61,60,59,58,57,56,55,54,53,52,51,50,48,47,46,45,44,42,41,40,38,
 37,35,34,32,31,29,27,25,23,20,18,14,10,0
};

////////////////////////////////////////////////////////////////////////////////
//  Setup
////////////////////////////////////////////////////////////////////////////////
void setup() {                                         // Begin setup


  Timer1.initialize(DimStep);                          // Start up the Timer1 timer
  attachInterrupt(1, zero_cross_detect, FALLING);      // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  pinMode(TriacPin[0], OUTPUT);                        // Set the Triac pin as output
  measure_half_period();                               // Initially measure the half period

  m_SyNetUniqueID.byte[1] = EEPROM.read(EEPROM_ID);
  m_SyNetUniqueID.byte[0] = EEPROM.read(EEPROM_ID+1);

  m_ManufacturerID.byte[1] = EEPROM.read(EEPROM_MANUFAC);
  m_ManufacturerID.byte[0] = EEPROM.read(EEPROM_MANUFAC+1);

  m_ProfileID.byte[1] = EEPROM.read(EEPROM_PROFILE);
  m_ProfileID.byte[0] = EEPROM.read(EEPROM_PROFILE+1);

  m_RevisionID.byte[1] = EEPROM.read(EEPROM_REVISION);
  m_RevisionID.byte[0] = EEPROM.read(EEPROM_REVISION+1);

  // Setup Radio
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
 
 	pinMode(PIN_POT1, INPUT);
	pinMode(PIN_POT2, INPUT);

	xbee.begin(19200);

  // Blink to signify app start
  blinkLED(5,100);
 
  // Synet Device Status Announce
  SendDeviceStatus(SYNET_API_DEVICESTATUS_ACTIVE);

	getPotChange();
} // end setup


////////////////////////////////////////////////////////////////////////////////
//  Loop
////////////////////////////////////////////////////////////////////////////////
void loop() {                                          // Main Loop

  if (m_ledCount++ > 10000)
  {
    m_ledCount = 0;
    blinkLED(1);
  }

  if ( millis() >= ResetPeriod ) {                     // Measure the half period every PeriodResync milliseconds to prevent drift
    measure_half_period();
  }

  // If there's a packet available coming in, process it...
  if( xbee.poll() ) {
			char byteIn; 
      /************************************************************************
       * Message Handling
       ***********************************************************************/

      switch( xbee.getSyNetAPI() ) {
      /////////////////////////////////////////////////////
      //  Bootload
      /////////////////////////////////////////////////////
        case SYNET_API_BOOTLOADTX: 
          byteIn = xbee.getNextDataByte();
          if (byteIn == SYNET_API_BOOTLOADTX_REBOOT)
          {
            digitalWrite(PIN_RESET, HIGH);
          }
          break;
      /////////////////////////////////////////////////////
      //  Device Status Request
      /////////////////////////////////////////////////////
        case SYNET_API_STATUSREQUEST:
          byteIn = xbee.getNextDataByte();
          if (byteIn == SYNET_API_STATUSREQUEST_STATUS)
          {
            SendDeviceStatus(SYNET_API_DEVICESTATUS_ACTIVE);
          } 
          else if (byteIn == SYNET_API_STATUSREQUEST_INFO)
          {
            SendDeviceStatus(SYNET_API_DEVICESTATUS_INFO);
          }
          break; 
      /////////////////////////////////////////////////////
      //  Catalog Request
      /////////////////////////////////////////////////////
        case SYNET_API_CATALOGREQUEST:
          byteIn = xbee.getNextDataByte();
          SendCatalogResponse(byteIn); 
          break;
      /////////////////////////////////////////////////////
      //  Parameter Request
      /////////////////////////////////////////////////////
        case SYNET_API_PARAMREQUEST:
          {
            uint8_t funcID = xbee.getNextDataByte();
            uint8_t paramID = xbee.getNextDataByte();
            SendParamResponse(funcID, paramID); 
            break;
          }
      /////////////////////////////////////////////////////
      //  Parameter Request
      /////////////////////////////////////////////////////
        case SYNET_API_INFOSET:
          {
            uint8_t infoID = xbee.getNextDataByte();
            uint8_t info1 = xbee.getNextDataByte();
            uint8_t info2 = xbee.getNextDataByte();
            SetInfo(infoID, info1, info2); 
            SendInfoResponse(infoID);
            break;
          } 
      /////////////////////////////////////////////////////
      //  Function Transmit From Controller
      /////////////////////////////////////////////////////
        case SYNET_API_FUNCTIONTRANSMIT:
          uint8_t functionID = xbee.getNextDataByte();
          uint16_t fade;
          uint8_t brightness;
          // Switch on the function identifier
          switch (functionID)
          { 
            case 1: // FadeLights
            {
              brightness = xbee.getNextDataByte();
              fade = xbee.getNextDataWord();
						  if (brightness >= 0 && brightness < 256)
							{
  							m_powerStep = (int)brightness;
							}
            } 
            break;
          }
          break;
      }
  }

  //
  // Handle the local dimmer
  //
	m_potDir = getPotChange();
	if( m_potDir != 0 )
	{
		if (m_potDir == 1)
		{
			StepLightUp();
		}

		if (m_potDir == 2)
		{
			StepLightDown();
		}
	}

 	delay(1);
 }

////////////////////////////////////////////////////////////////////////////////
//  Device functions
////////////////////////////////////////////////////////////////////////////////
void measure_half_period() {
  zero_cross = 0;                                      // Clearing this here increases the accuracy of the measurement
  byte F = 0;                                          // Frequency counter counter  ;)
  while ( F < 4 ) {                                    // This loop takes 4 zero cross samples
    if ( zero_cross ) {                                // Only run if a zero cross is detected
      ZeroXTime[F] = micros();                         // Set the new current zero cross time in micros()
      zero_cross = 0;                                  // Reset zero_cross
      F++;                                             // Bump the counter for the next sample
    }
  }                                                    // Now we calc the length of each DimStep
  DimStep = (((ZeroXTime[1]-ZeroXTime[0]) + (ZeroXTime[2]-ZeroXTime[1]) + (ZeroXTime[3]-ZeroXTime[2])) / 3) / DimRes;
  Timer1.attachInterrupt(fire_triacs, DimStep);        // (Re)Associate fire_triacs() with the Timer1 interrupt and the latest DimStep period
  ResetPeriod = ResetPeriod + PeriodResync;            // Set the next time when we'll measure the half period again
}
  
void zero_cross_detect() {                             // function to be fired at the zero crossing
  zero_cross = 1;                                      // set a variable that's picked up later
  DimStepCounter = 0;                                  // Reset the step counter for the next round of triac firings
}

byte getPotValue()
{
	return ( 0 | (digitalRead(PIN_POT1)<<1) | digitalRead(PIN_POT2) );
}

// 0 = no change
// 1 = forward
// 2 = backwards
int getPotChange()
{
	int index = getPotValue();
	int prevIndex = m_prevPotIndex;
	int retVal = 0;


		
	if (index != prevIndex)
	{

		if ( 	(index == 0 && prevIndex == 1) ||
					(index == 2 && prevIndex == 0) ||
					(index == 3 && prevIndex == 2) ||
					(index == 1 && prevIndex == 3) )
		{
			retVal = 1;
		} else if
				( (index == 0 && prevIndex == 2) ||
					(index == 1 && prevIndex == 0) ||
					(index == 2 && prevIndex == 3) ||
					(index == 3 && prevIndex == 1) )
		{
			retVal = 2;
		}

		//out[0] = prevIndex;
		//out[1] = index;
		//SendDataPacket(out, 2);
	}

	if (retVal != 0)
	{
		m_prevPotIndex = index;
	}

	return retVal;
}

void StepLightUp()
{
	if (m_powerStep <= (255-m_powerInc))
	{
		m_powerStep+=m_powerInc;
	}
}

void StepLightDown()
{
	if (m_powerStep >= m_powerInc)
	{
		m_powerStep-= m_powerInc;
	}
}

void fire_triacs() {                                   // Called every DimStep (Timer1 interrupt, checks FireTriac[n] and fires if it's time
  if ( FireTriac[0] == DimStepCounter ) {              // Is it time to fire?
    digitalWrite(TriacPin[0], LOW);                   // Fire the Triac mid-phase
    delayMicroseconds(2);
    digitalWrite(TriacPin[0], HIGH);                    // Turn off the Triac gate (Triac will not turn off until next zero cross)

		if( m_powerStep >= 0 && m_powerStep <= 255 )
		{
  		FireTriac[0] = PowerMap[m_powerStep];
		}

  }
  DimStepCounter++;                                    // This counter increments every time fire_triacs runs
}


void WriteEEPROMBytes(int p_loc, byte p_b1, byte p_b2)
{
  EEPROM.write(p_loc, p_b1);
  EEPROM.write(p_loc+1, p_b2);
}

void SetInfo(byte infoID, byte info1, byte info2) {
  switch (infoID) {
    case INFO_API_ID:
      WriteEEPROMBytes(EEPROM_ID, info1, info2);
      break;
    case INFO_API_MANUFAC:
      WriteEEPROMBytes(EEPROM_MANUFAC, info1, info2);
      break;
    case INFO_API_PROFILE:
      WriteEEPROMBytes(EEPROM_PROFILE, info1, info2);
      break;
    case INFO_API_REVISION:
      WriteEEPROMBytes(EEPROM_REVISION, info1, info2);
      break;
  }
}

void blinkLED(int p_num)
{
  blinkLED(p_num, 200);
}

void blinkLED(int p_num, int p_delay)
{
  for( int i=0; i<p_num; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(p_delay);
    digitalWrite(PIN_LED, LOW);
    delay(p_delay);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Synet Functions
////////////////////////////////////////////////////////////////////////////////

//
// Send Info Response
//
void SendInfoResponse(byte p_infoID)
{
  xbee.startNewPacket();
  xbee.addDataByte(SYNET_API_INFOSETRESPONSE);
  xbee.addDataByte(p_infoID);
  xbee.sendDataPacket();
}

//
// Send device status
//
void SendDeviceStatus(char p_status)
{

  xbee.startNewPacket();
  xbee.addDataByte(SYNET_API_DEVICESTATUS);
  xbee.addDataByte(p_status);

  if (p_status == SYNET_API_STATUSREQUEST_INFO)
  {
    xbee.addDataUnion(m_SyNetUniqueID);
    xbee.addDataUnion(m_ManufacturerID);
    xbee.addDataUnion(m_ProfileID);
    xbee.addDataUnion(m_RevisionID);
  }
  xbee.sendDataPacket();
}

//
// Send parameter response
//
void SendParamResponse(byte fType, byte pType)
{
  // Setup start
  xbee.startNewPacket();
  xbee.addDataByte(SYNET_API_PARAMRESPONSE);

  if (fType == 0x01) {
  /********************************************************
   * FadeLight
   *******************************************************/
    if (pType == 0x01) {
      // FadeLight brightness
      xbee.addDataByte(1); // Function ID
      xbee.addDataByte(1); // Param ID
      xbee.addDataByte(1); // Parameter Type: byte
      xbee.addDataByte(0); // Validation Type: unsigned full
      xbee.addDataString("Brightness");
      xbee.sendDataPacket();
    }

    if (pType == 0x02) {
      // FadeLight brightness
      xbee.addDataByte(1); // Function ID
      xbee.addDataByte(2); // Param ID
      xbee.addDataByte(2); // Parameter Type: word
      xbee.addDataByte(1); // Validation Type: unsigned range
      xbee.addDataWord(0); // Validation Min: 0
      xbee.addDataWord(10000); // Validation Max: 10,000ms
      xbee.addDataString("FadeTime");
      xbee.sendDataPacket();
    }
  }
}

//
// Send catalog response
//
void SendCatalogResponse(byte p_type)
{
  // Setup start 
  xbee.startNewPacket();
  xbee.addDataByte(SYNET_API_CATALOGRESPONSE);
  xbee.addDataByte(1); // Number of entries

  if (p_type == 0x00) {
    xbee.addDataByte(0);
    xbee.sendDataPacket();
  }

  if (p_type == 0x01) {
    // void FadeLights(word r, word b, word g word fadeTime);
    xbee.addDataByte(1); // Function ID
    xbee.addDataByte(2); // Num Params
    xbee.addDataByte(0); // Return Type: void
    // Params
    xbee.addDataByte(1); // Byte Brightness
    xbee.addDataByte(2); // Word FadeTime
    
    xbee.addDataString("Fadelight");
    xbee.sendDataPacket();
  }
}


