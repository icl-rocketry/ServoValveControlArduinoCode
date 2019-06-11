/*************************************************** 

 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//Calibrated for the FS5115M we have
//#define SERVOMIN 490 // this is the 'minimum' pulse length in us
#define SERVOMIN 480 // this is the 'minimum' pulse length in us
//#define SERVOMAX 2270 // this is the 'maximum' pulse length in us
#define SERVOMAX 2300 // this is the 'maximum' pulse length in us
#define MAXBYTESREADPERTICK 100

uint32_t servoMinPWMs[] = {480, 0};
uint32_t servoMaxPWMs[] = {2300, 700};
// our servo # counter
uint8_t servonums[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
double servoAngle[16]; //Current desired servo angles in degrees
byte *commands[16]; //Array of commands arrays pointers, index is the servo num
int cmdIndex[16]; //Indexes of command currently being executed by each servo
int cmdLength[16]; //Lengths of commands bytes currently available to each servo

void setup() {
  Serial.begin(57600);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //delay(10);
  Serial.println("Ready");
}

void setServoAngle(uint8_t n, double angle, unsigned int minPulseLength, unsigned int maxPulseLength, double angleRange){
  double pulselen = map(angle, 0, angleRange, (double)minPulseLength, (double)maxPulseLength);
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
  while (Serial.available() > 0 && readThisTick < MAXBYTESREADPERTICK) {
                readThisTick += 1;
                static String inString = "";    // string to hold input
                static size_t readBufferPos;              // position of next write to buffer
                if(!readingCmd){ //Will be sent the length to read followed by \n
                    // read the incoming byte:
                    char c = char(Serial.read()); //Read ASCII char from serial
                    inString += c; //add to the buffer
                    if (c == '\n') {            // \n means "end of message"
                        //cmdBuffer = new byte[stringToLong(inString)]; //Message is length in bytes of command list, create byte array buffer for storing the commands
                        cmdLen = inString.toInt();
                        inString = "";                // reset buffer
                        i=0;
                        readingCmd = true;
                    }
                }
                else { //Read set of commands
                    cmdBuffer[i] = Serial.read();
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
                              Serial.println("FAILED to allocate memory for storing commands!");
                              Serial.flush();
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
        byte cmd = cmds[cmdI] & 0x0F;
        if (cmd == 0x00){ //Command to move servo to desired angle
           uint32_t argRaw = (((uint32_t) cmds[cmdI+1]) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode to uint
           servoAngle[servonumI] = (((double)argRaw*180) / 65535.0); //Set the servo angle
           //Send the updated servo angle to whoever is listening on COM in the format "s<Servonum>a<angle>"
           String strAngle = "s";
           strAngle +=servonumI;
           strAngle +="a";
           strAngle += servoAngle[servonumI];
           Serial.println(strAngle);
           Serial.flush();
           cmdIndex[servonumI] = cmdI+3;
        } else if(cmd == 0x01){ //Command for servo to wait given number of millis
          //argRaw is time in millis to wait
          uint32_t argRaw = (((uint32_t) cmds[cmdI+1]) << 8) + ((uint32_t) cmds[cmdI+2]); //Decode to uint
          if(waitTimeStart[servonumI] < 0){
            waitTimeStart[servonumI] = millis();
          }
          if (millis() > waitTimeStart[servonumI] + argRaw){
            waitTimeStart[servonumI] = -1;
            cmdIndex[servonumI] = cmdI+3;
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
            cmdIndex[servonumI] = cmdI+3;
            reloop = true; //Execute next command in same tick
          }
        }
      }
    }
  }
}

void loop() {
  handleInput();
  doCommandHandling();
  for (int servonumI=0;servonumI<16;servonumI++){
    setServoAngle(servonums[servonumI],servoAngle[servonumI],servoMinPWMs[servonums[servonumI]],servoMaxPWMs[servonums[servonumI]],180);
  }
}
