/* Arduino LANC Controller
 * 2017 Zach Glenwright
 * http://www.gullswingmedia.com
 *
 * ...with a bit of code from all over the place - such as...
 *
 * * Recieving X10 signals using a detuned 315Mhz RF reciever, as described by BroHogan here:
 * http://brohogan.blogspot.com/2010/10/receiving-x10-rf-transmissions.html
 *
 * * Uses the X10rf (X10ex) library (to properly recieve X10 RF signals) from:
 * http://load-8-1.blogspot.com/2010_06_01_archive.html
 * * ...also uses a modified X10rf.h file, with greatly improved signal detection timings, found here:
 * https://www.dropbox.com/s/h0t9la14i7q4xpb/X10rf.h?dl=1
 *
 * * Original LANC Control Code (that this program is derived from) by Martin Koch - for the Canon codebase:
 * http://controlyourcamera.blogspot.com.au/2011/02/arduino-controlled-video-recording-over.html
 *
 * Uses a slightly modified version of the LANC library (for some reason, I can't get this codebase to work
 * with Canon cameras, despite the claims that it does, but it works quite well on Sony cameras!)
 * https://github.com/EdsterG/LANC_Library
 *
 * "LANC" is a registered trademark of SONY.
 * CANON calls their LANC compatible port "REMOTE".
 *
 * v. 0.1a - 11-12-17 (initial upload) - initial upload, but not initial version.  A lot of trial and error
 * (and error, and...) led to this code base, which works fairly well with Sony camcorders.  For some reason,
 * I can't get it to work with Canon cameras, but I'm working on it...
 */

#include <X10rf.h>

#define lightPin 4 // the pin to activate the VideoLight II board

#define _cmdPin 7
#define _lancPin 11

#define bitWidth 104

int lightBoard = 0 ; // if we're using a Video Light II board, set this to 1

byte LANC_Frame[8];

const int showDebug = 1; // set this to 1 to show debug messages

int isRecording = 0; // set to not recording to initialize the variable
int isIgnoringSensors = 0;

int useCanonMode = 0 ; // set to 1 to use the Canon LANC commands instead of the Sony ones
int recordDelay = 3000; // delay between turning camera on and hitting RECORD

int operationTimeout = 12500 ; // operation timeout before quitting out of loops that may hang

volatile int sensorDetected = 0; // the RF sensor changes this when it detects an event from an X10 source

unsigned long REC_TO_CTR = 30000; // timer before stopping record
unsigned long STBY_TO_CTR = 17500; // after record timer before shutting camera down

// CANON LANC BITS TO SEND TO LANC (for some reason, the Sony procedure isn't timed correctly... yet.)
boolean REC[] = {0,0,0,1,1,0,0,0,   0,0,1,1,0,0,1,1}; //18 33
boolean POWER_OFF[] = {0,0,0,1,1,0,0,0,   0,1,0,1,1,1,1,0}; //18 5E

void processRfCommand(char house, byte unit, byte command, bool isRepeat){
	if (!sensorDetected) {
		if (isIgnoringSensors == 0) { // if we aren't ignoring sensor input
			sensorDetected = 1;

			if (showDebug) {
				Serial.print("RF Signal Detected! -> ");
				Serial.print(house);
				Serial.print("-");
				Serial.println(unit, DEC);
			}
		}
		else { // if we are ignoring sensor input
			if (showDebug) Serial.println("Sensor detected, but ignoring sensors for the time being!");
			isIgnoringSensors++;
		}
	}
}

X10rf x10rf = X10rf(0, 2, processRfCommand); // set up the above function to trigger on X10 events

void setup() {
  if (showDebug) {
    Serial.begin(9600); // if you have the system set to show debug messages, start the Serial connection
    Serial.println("This is the COMBINED code base");
    Serial.println("Last Modified: 12/27/17 - B -");
	Serial.println("Added routine to block sensor inputs for 30 seconds");
	Serial.println("after event is captured to cut down on false extras...");
	Serial.println("...hopefully!");
	Serial.println("-----------------------------------------------");
    Serial.print("The record delay is set to ");
    Serial.println(REC_TO_CTR);
    Serial.print("The standby delay is set to ");
    Serial.println(STBY_TO_CTR);
    Serial.println("-----------------------------------------------");
  }

  x10rf.begin();

  STBY_TO_CTR = REC_TO_CTR + STBY_TO_CTR; // add the standby timer to the rec timer for the entire timer

  if (lightBoard) {
	pinMode(lightPin, OUTPUT);
	digitalWrite(lightPin, 0); // turn the light board off to initialize it!
  }

  pinMode(_lancPin, INPUT);
  digitalWrite(_lancPin, HIGH);
}

void turnCameraOn() {
	for (int i = 0; i < 3; i++) {
       digitalWrite(7, 1); // set lanc pin to +5V to kick the camera back on
       delay(750); // allow the camera to come up to power
       digitalWrite(7, 0); // set lanc pin back to +0V, so the next step can start...
	}
}

void update(int type, int code, int doVerify){
  //Write the first 2 bytes
  for (int reps = 1; reps < 6; reps++) {
    _nextFrame();     //Find the beginning of the next frame
    _writeByte(type); //Send Sub-Command for Byte 1
    _nextByte();      //Find the beginning of the next byte
    _writeByte(code); //Send the actual Command-Code, Byte 1
  }

  // if you need to verify the camera is working (for recording), then check the return...
  if (doVerify) {
	for (int repeats = 0; repeats < 25; repeats++) {
		_nextFrame(); // advance 25 LANC frames before asking for the status
	}

	//Read the current LANC status (don't send any command, just read the LANC line)
	for (int position = 0; position<8; position++){
		_nextByte();
		LANC_Frame[position]=_readByte();
	}
  }
}

void _nextFrame(){
	bool longEnough=false;
	int count;

	//Search for a 3ms gap (at least), being HIGH continuosly.
	//This worked extraordinarily well in PAL/NTSC Sony video cameras.
	//You may need to reduce the loop limit for your camera, hopefully not.

	unsigned long startTimer = millis(); // start the anti-hang timer

	while (!longEnough){
		if ((millis() - startTimer) < operationTimeout) {
			for (count=0; count<100; count++){
				delayMicroseconds(25);
				if (digitalRead(_lancPin)==LOW)
					break;
			}

			if (count==100) //Checks if total gap length greater than 2500 uS
				longEnough=true;
		}
		else {
			if (showDebug) Serial.println("_nextFrame() hung waiting for the 3ms gap");
			break;
		}
	}

	startTimer = millis(); // re-start the anti-hang timer

	//Now wait till we get the first start bit (low)
	while (digitalRead(_lancPin)==HIGH) {
		if ((millis() - startTimer) < operationTimeout) {
			delayMicroseconds(20);
		}
		else {
			if (showDebug) Serial.println("_nextFrame() hung waiting for start bit");
			break;
		}
	}
}

void _nextByte(){
	//Now wait till we get the first start bit (low)

	while(1) {
		//this will look for the first LOW signal with abou 5uS precission
		while (digitalRead(_lancPin)==HIGH) {
			delayMicroseconds(2);
		}

		break; // the code below kept hanging the board, because it wouldn't find the LOW

		/*
		//And this guarantees it was actually a LOW, to ignore glitches and noise
		delayMicroseconds(7);
		if (digitalRead(_lancPin)==LOW)
			break;
		*/
	}
}

void _writeByte(byte value){
	//Note: Bits are inverted in the LANC protocol
	//      GND(0v) = 1 and VCC(5v) = 0
	//Note that timming here is critical, Ariel Rocholl used a logic analyzer
	//to reverse engineering the right delay times. This guarantees closer to
	//LANC standard so it increases chances to work with any videocamera, but
	//you may need to fine tune these numbers for yours, hopefully not.
	pinMode(_lancPin, OUTPUT);

	delayMicroseconds(bitWidth-20); //Ignore start bit

	for (int i = 0; i < 8; i++){
		boolean bit = value & 0x1;
		digitalWrite(_lancPin,!bit); //Remember, LANC bits are inverted
		value >>= 1;
		delayMicroseconds(bitWidth-10);
	}

	pinMode(_lancPin, INPUT);
	digitalWrite(_lancPin,HIGH);
}

byte _readByte(){
	//Timming here is not as critical as when writing, actually it is a bit delayed
	//when compared to start edge of the signal, but this is good as it increases
	//chances to read in the center of the bit status, thus it is more immune to noise
	//Using this I didn't need any hardware filter such as RC bass filter.
	unsigned char nByte=0;

	delayMicroseconds(bitWidth*1.5); //ignore start bit

	for (int i = 0; i<8; i++){
		nByte|= digitalRead(_lancPin) << i;
		delayMicroseconds(bitWidth-10);
	}

	delayMicroseconds(bitWidth*0.6);

	nByte = nByte ^ 255;  //invert bits, we got LANC LOWs for logic HIGHs
	return nByte;
}

void loop() {
	if (sensorDetected) {
		if (showDebug) Serial.println("Turning power on");
		turnCameraOn(); // kick the camera out of STBY mode and get it ready to record
		delay(recordDelay); // wait recordDelay (set at the top of the script) seconds before sending LANC commands to camera

		if (showDebug) Serial.println("Hitting Record");
		toggleRec(); // tell camera to start recording
		verifyRec(1); // verify that the camera is recording

		sensorDetected = 0; // set sensorDetected to 0 to clear the notification

		if (lightBoard) digitalWrite(lightPin, 1); // turn the light on if we have the Video Light II board enabled

		timeout(); // do the REC PAUSE countdown timer


		if (lightBoard) digitalWrite(lightPin, 0); // turn the light off if we have the Video Light II board enabled

		for (int i = 1 ; i < 10; i++){
			if (!useCanonMode) {
				update(0x18, 0x5E, 0); // turn the camera off
			}
			else {
				lancCommandCanon(POWER_OFF);
			}
		}

		delay(3000);

		isRecording = 0 ; // just in case isRecording is set wrong, restore it to "non-recording" state after the process ends
	}

	if (showDebug) Serial.println("Waiting for an X10 event...");
	delay(1000);
}

void toggleRec(){
	if (!useCanonMode) {
		update(0x18, 0x33, 1); // send the Sony command to REC to the camera
	}
	else {
		lancCommandCanon(REC); // send the Canon command to REC to the camera
		delay(800);
		update(0x00, 0x00, 1); // send the Sony command to REC to the camera
		delay(300);
	}
}

void verifyRec(int typeOfRecording) {
	int failedTries = 0 ;

	while(1) {
		if (LANC_Frame[4] == 0x04 || LANC_Frame[4] == 0x14 ) {
			if (typeOfRecording == 1) {
				if (LANC_Frame[4] == 0x04) {
					Serial.println("We're supposed to be recording and the camera says we are");
					isRecording = 1;
					break;
				}
			else {
				Serial.println("We're supposed to be recording and we aren't");
				toggleRec();
				failedTries++;
			}
		}
		else if (typeOfRecording == 0) {
			if (LANC_Frame[4] == 0x14) {
				Serial.println("We're supposed to be in STBY and the camera says we are");
				isRecording = 0;
				break ;
			}
			else {
				Serial.println("We're supposed to be in STBY and we aren't");
				toggleRec();
				failedTries++;
			}
		}
	}
	else { // if the LANC stream doesn't contain 04 or 14 in hex, then refresh the stream and try again!
		update(0x00, 0x00, 1); // send the Sony command to REC to the camera
		delay(500);
	}

	if (failedTries > 10) {
		if (!useCanonMode) {
			useCanonMode = 1;
			if (showDebug) Serial.println("Switching to Canon compatibility mode for this session");
		}
	}
  }
}

void lancCommandCanon(boolean lancBit[]) {
	for (int cmdTotalRepeatCount = 0 ; cmdTotalRepeatCount < 4; cmdTotalRepeatCount++) {
		//_nextFrame();
		while (pulseIn(_lancPin, 1) < 5000) { }

		//0 after long pause means the START bit of Byte 0 is here
		delayMicroseconds(bitWidth - 8);  //wait START bit duration

		// WRITE FIRST PART OF COMMAND
		for (int i=7; i>-1; i--) { //Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
			digitalWrite(_cmdPin, lancBit[i]);  //Write bits.
			delayMicroseconds(bitWidth - 8);
		}

		//Byte 0 is written now put LANC line back to +5V
		digitalWrite(_cmdPin, 0);
		delayMicroseconds(10); //make sure to be in the stop bit before byte 1

		//_nextByte();
		while (digitalRead(_lancPin)) { }

		//0V after the previous stop bit means the START bit of Byte 1 is here
		delayMicroseconds(bitWidth - 8);  //wait START bit duration

		// WRITE SECOND PART OF COMMAND
		for (int i=15; i>7; i--) {
			digitalWrite(_cmdPin,lancBit[i]);  //Write bits
			delayMicroseconds(bitWidth - 8);
		}

		digitalWrite(_cmdPin, 0); //Byte 1 is written now put LANC line back to +5V
		cmdTotalRepeatCount++;  //increase repeat count by 1
	}
}

/*
void ignoreSensors() {
	unsigned long currentTime = millis();
	int waitTime = 25000;

	while((currentTime + waitTime) > millis()) {
		// wait until the time expires before checking for another event
	}

	isIgnoringSensors = 0;
}
*/

void timeout() {
  unsigned long TO_CTR; // beginning of timeout counter
  unsigned long currentTime; // the current time

  TO_CTR = millis();

  while((millis() - TO_CTR) < STBY_TO_CTR){
    currentTime = millis();

    if ((currentTime - TO_CTR) < REC_TO_CTR){
      if (showDebug) {
        Serial.print("Waiting ");
        Serial.print((REC_TO_CTR - (currentTime - TO_CTR))/1000);
        Serial.println(" seconds for REC PAUSE timeout");
      }
    }
    else {
      if (isRecording) {
		delay(500);
        toggleRec();
        verifyRec(0);
      }

      if (showDebug) {
        Serial.print("Waiting ");
        Serial.print((STBY_TO_CTR - (currentTime - TO_CTR))/1000);
        Serial.println(" seconds for STBY PAUSE timeout");
      }
    }

	if ((millis() - TO_CTR) > 45000) {
		if (isIgnoringSensors != -1) {
			isIgnoringSensors = -1;
			if (showDebug) Serial.println("Ignoring the next sensor event");
		}
	}

    if (sensorDetected) {
      if ((currentTime - TO_CTR) > REC_TO_CTR){
        if (showDebug) Serial.println("Signal! / STBY -> REC / Start over"); // we're in STBY mode, so start the recording again and reset the timer
        toggleRec();
        verifyRec(1);
      }
      else {
        if (showDebug) Serial.println("Signal! / Starting timer over"); // we're in REC mode, so we only need to start the timer again
      }

      TO_CTR = millis(); // reset countdown timer
      sensorDetected = 0; // set sensorDetected to 0 to clear the notification
    }

    //delay(500);
  }
}