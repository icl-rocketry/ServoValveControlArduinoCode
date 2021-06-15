/*************************************************** 

 ****************************************************/

//Made for ServoValveController V1.2

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

#define MAXBYTESREADPERTICK 100
//Which Serial to use for communication. "Serial" for normal USB/etc serial, or "mySerial" for SoftwareSerial
#define SERIALOUT mySerial

//Maximum angle used in encoded scheme for communication to commanding program. Must match or angles commanded will be wrong. (Angle is sent as an integer number of 1/65535 ths of this number)
#define MAX_ENCODED_ANGLE 360

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//Calibrated for the FS5115M we have
uint32_t servoMinPWMs[] = {480, 480}; //PWMs for servo 0 degrees position
uint32_t servoMaxPWMs[] = {2300, 2300}; //PWMs for servo max position
double servoMaxAngles[] = {180, 180}; //Servo angle for max PWM position
// our servo # counter
uint8_t servonums[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
double servoAngle[16]; //Current desired servo angles in degrees
byte *commands[16]; //Array of commands arrays pointers, index is the servo num
int cmdIndex[16]; //Indexes of command currently being executed by each servo
int cmdLength[16]; //Lengths of commands bytes currently available to each servo
SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //delay(10);
  //Serial.println("Ready");
  SERIALOUT.println("Ready");
}

void setServoAngle(uint8_t n, double angle, unsigned int minPulseLength, unsigned int maxPulseLength, double angleRange){
  double pulselen = constrain(map(angle, 0, angleRange, (double)minPulseLength, (double)maxPulseLength), minPulseLength, maxPulseLength);
  //Serial.println(pulselen);
  setServoPulse(n,pulselen);
}

// you can use this function if you'd like to set the pulse length in microseconds
// e.g. setServoPulse(0, 1) is a ~1 microsecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  //Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  //Serial.print(pulselength); Serial.println(" us per bit"); 
  //pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  //Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

boolean readingCmd = false;
  int i=0; //Index to write next command received
  int cmdLen=0;
  int subCmdBufferIndex = 0; //Index to write to
  int readThisTick = 0;
void handleInput(){
  readThisTick = 0;
  byte cmdBuffer[300]; //Max 100 cmds per command list
  byte subCmdBuffer[300]; //Max 100 cmds per servo per command list
  while (SERIALOUT.available() > 0 && readThisTick < MAXBYTESREADPERTICK) {
                readThisTick += 1;
                static String inString = "";    // string to hold input
                static size_t readBufferPos;              // position of next write to buffer
                if(!readingCmd){ //Will be sent the length to read followed by \n
                    // read the incoming byte:
                    char c = char(SERIALOUT.read()); //Read ASCII char from serial
                    inString += c; //add to the buffer
                    if (c == '\n') {            // \n means "end of message"
                        //cmdBuffer = new byte[stringToLong(inString)]; //Message is length in bytes of command list, create byte array buffer for storing the commands
                        String isReadyStr = "isReady\n";
                        if(inString == isReadyStr){
                          inString = "";                // reset buffer
                          i=0;
                          SERIALOUT.println("Ready");
                          readingCmd = false;
                        }
                        else {
                          cmdLen = inString.toInt();
                          inString = "";                // reset buffer
                          i=0;
                          readingCmd = cmdLen > 0;
                        }
                    }
                }
                else { //Read set of commands
                    cmdBuffer[i] = SERIALOUT.read();
                    i+=1;
                    if (i == cmdLen){ //Have finished reading the command list, now to write it so it gets executed
                        readingCmd = false;
                        //Sub list of commands for servo (buffer)
                        for (int servoNumI=0;servoNumI<16;servoNumI++){ //Go through each servo num and search for commands
                          for (int k=0;k<cmdLen;k+=3){ //Go through all commands received and see if they match this servo
                            int servoNumOfCmd = (int) (cmdBuffer[k] & 0xF0) >> 4;
                            if ((int) (cmdBuffer[k] & 0xF0) >> 4 == servonums[servoNumI]){ //If servo num of this command matches that we are searching for cmds for
                              //Write this command to the sub list of commands for this servo
                              subCmdBuffer[subCmdBufferIndex] = cmdBuffer[k];
                              subCmdBuffer[subCmdBufferIndex+1] = cmdBuffer[k+1];
                              subCmdBuffer[subCmdBufferIndex+2] = cmdBuffer[k+2];
                              subCmdBufferIndex += 3;
                            }
                          }
                          if (subCmdBufferIndex > 0){ //If any new commands specified for this servo
                            if(commands[servoNumI] != NULL){
                              delete [] commands[servoNumI]; //Clear current commands array from memory
                            }
                            //Dynamically allocate a new commands array for this servo
                            //Serial.println("Allocating memory for commands array");
                            //Serial.flush();
                            byte *cmdsArr = new byte[subCmdBufferIndex](); //subCmdBufferIndex is number of commands this servo received
                            if(cmdsArr==NULL){
                              SERIALOUT.println("FAILED to allocate memory for storing commands!");
                              SERIALOUT.flush();
                            }
                            for (int z=0;z<subCmdBufferIndex;z++){
                              cmdsArr[z] = subCmdBuffer[z]; //Copy from sub cmd buffer into the commands array for this servo
                            }
                            commands[servoNumI] = cmdsArr; //Set the pointer to commands for this servo to point to the created array
                            cmdIndex[servoNumI] = 0; //Reset index of command this servo is currently executing
                            cmdLength[servoNumI] = subCmdBufferIndex;
                            subCmdBufferIndex = 0; //Clear the index ready to check commands for next servo
                            i = 0;
                          }
                        }
                    }
                }
        }
}

long waitTimeStart[16] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,}; //If servo is waiting 
void doCommandHandling(){
  for (int servonumI=0;servonumI<16;servonumI++){
    byte *cmds = commands[servonumI];
    if (cmds == NULL){ //do not just read random bits of memory..., if pointer null do not do commands
      continue;
    }
    boolean reloop = true;
    while (reloop){
      reloop = false;
      int cmdI = cmdIndex[servonumI];   
      if (cmdI < cmdLength[servonumI]){
        byte cmd = cmds[cmdI] & 0x0F; //Extract the last 4 bits of the command which denote what to do (First 4 bits were used to address which servo)
        if (cmd == 0x00){ //Command to move servo to desired angle
           uint32_t argRaw = (((uint32_t) cmds[cmdI+1]) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode the two bytes following the command to a uint.
           //Arg raw is the angle
           servoAngle[servonumI] = (((double)argRaw*MAX_ENCODED_ANGLE) / 65535.0); //Set the servo angle
           //Send the updated servo angle to whoever is listening on COM in the format "s<Servonum>a<angle>"
           String strAngle = "s";
           strAngle +=servonumI;
           strAngle +="a";
           strAngle += constrain(servoAngle[servonumI],0,servoMaxAngles[servonumI]);
           SERIALOUT.println(strAngle);
           SERIALOUT.flush();
           cmdIndex[servonumI] = cmdI+3; //Progress to next command
        } else if(cmd == 0x01){ //Command for servo to wait given number of millis
          //argRaw is time in millis to wait
          uint32_t argRaw = (((uint32_t) cmds[cmdI+1]) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode to uint
          if(waitTimeStart[servonumI] < 0){
            waitTimeStart[servonumI] = millis();
          }
          if (millis() > waitTimeStart[servonumI] + argRaw){
            waitTimeStart[servonumI] = -1;
            cmdIndex[servonumI] = cmdI+3; //Progress to next command
            reloop = true; //Execute next command in same tick
          }
        } else if(cmd == 0x02){ //Command for servo to wait given number of seconds
          //argRaw is time in sec to wait
          uint32_t argRaw = (((uint32_t) cmds[cmdI+1]) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode to uint
          if(waitTimeStart[servonumI] < 0){
            waitTimeStart[servonumI] = millis();
          }
          if (millis() > waitTimeStart[servonumI] + argRaw*1000){
            waitTimeStart[servonumI] = -1;
            cmdIndex[servonumI] = cmdI+3; //Progress to next command
            reloop = true; //Execute next command in same tick
          }
        }
        else if(cmd == 0x04){ //Command for servo to wait for pin to go high / go low
          uint32_t pinNumber = (((uint32_t) (cmds[cmdI+1]) & 0x7F) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode to uint (last 7 bits of first byte and all 8 bits of second byte)
          byte triggerOnHighOrLow = cmds[cmdI+1] && 0x80; //Left most bit of first argument byte is whether to trigger on high signal (1) or low signal (0) 
          pinMode(pinNumber, INPUT);
          int val = digitalRead(pinNumber);
          if ((triggerOnHighOrLow > 0 && val == HIGH) || (triggerOnHighOrLow < 1) && val == LOW){ //Trigger commands
            String strMessage = "mPin ";
            strMessage += pinNumber;
            strMessage +=" condition triggered";
            SERIALOUT.println(strMessage);
            SERIALOUT.flush();
            cmdIndex[servonumI] = cmdI+3; //Progress to next command
            reloop = true; //Execute next command in same tick
          }
        }
        else if(cmd == 0x05) { //Command for servo to loop it's command list
          String strMessage = "mServo ";
          strMessage += servonumI;
          strMessage +=" restarting command list";
          SERIALOUT.println(strMessage);
          SERIALOUT.flush();
          cmdIndex[servonumI] = 0; //Progress to next command
          reloop = true; //Execute next command in same tick
        }
      }
    }
  }
}

void loop() {
  handleInput();
  doCommandHandling();
  for (int servonumI=0;servonumI<16;servonumI++){
    setServoAngle(servonums[servonumI],constrain(servoAngle[servonumI],0,servoMaxAngles[servonumI]),servoMinPWMs[servonums[servonumI]],servoMaxPWMs[servonums[servonumI]],servoMaxAngles[servonumI]);
  }
}
