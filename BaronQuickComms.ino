
#include <Max3421e.h> 
#include <Usb.h> 
#include <AndroidAccessory.h> 

// Wheel encoder input pins
const int LW_ENCODER = 9;
const int RW_ENCODER = 10;

// IR Sensor input pins
const int IRS_FRONT = 11;
const int IRS_LFRONT = 12;
const int IRS_RFRONT = 13;
const int IRS_LEFT = 14;
const int IRS_RIGHT = 15;

// Right side motors drive output pins
// const int EN_R = 8; // Right Side:  Half Bridge 1 enable
// const int MC1_R = 9; // Right Side:  Motor Control 1
// const int MC2_R = 10; // Roght Side:  Motor Control 2

// Left Side Motors drive output pins
// const int EN_L = 2; // Left Side:  Half Bridge 1 enable
// const int MC1_L = 3; // Left Side:  Motor Control 1
// const int MC2_L = 4; // Left Side:  Motor Control 2
 
 // Note that the Left side motor pins are each exactly 2 more than their
// corresponding right side counterpart.  This lets us define an offset to 
// select a motor
const int LEFT_MOTOR = 2;
const int RIGHT_MOTOR = 8;

// The pins are in the same order
const int EN = 0; // Left Side:  Half Bridge 1 enable
const int MC1 = 1; // Left Side:  Motor Control 1
const int MC2 = 2; // Left Side:  Motor Control 2
 
// Input message buffer (global avoid reallocation on loop)
byte rcvmsg[3]; 

// Output message buffer
byte sndmsg[3];

// Start byte for reading input message
const byte START_BYTE = -2;

// How many milliseconds between I/O reads/writes?
const int LOOP_DELAY = 10;

// How many loops between ADK comms quierys?
const int ADK_INTERVAL = 1;
// How mnay loops between Wheel encoder queries?
const int WE_INTERVAL = 1;
// How many loops between IR sensor queries?
const int IR_INTERVAL = 5;

int adkCounter = 0;
int weCounter = 0;
int irCounter = 0;

// Output message types
const byte OUTPUT_IR = -3;
const byte OUTPUT_WE = -4;

// At what encode sensor reading level do we consider a wheel blocked?
const int WE_THRESH = 200;

// ADK device ID
AndroidAccessory acc("Manufacturer", 
                     "Model", 
                     "Description", 
                     "Version", 
                     "URI", 
                     "Serial"); 
                     

void setup(){
  // Turn on the ADK
  acc.powerOn(); 
  
  // set up the sensor pins as inputs
   pinMode(LW_ENCODER, INPUT);
   pinMode(RW_ENCODER, INPUT);
   pinMode(IRS_FRONT, INPUT);
   pinMode(IRS_LFRONT, INPUT);
   pinMode(IRS_RFRONT, INPUT);
   pinMode(IRS_LEFT, INPUT);
   pinMode(IRS_RIGHT, INPUT);
  
  
   // set up the h-bridge pins as outputs
    pinMode(RIGHT_MOTOR + MC1, OUTPUT);
    pinMode(RIGHT_MOTOR + MC2, OUTPUT);
    pinMode(RIGHT_MOTOR + EN, OUTPUT);
    pinMode(LEFT_MOTOR + MC1, OUTPUT);
    pinMode(LEFT_MOTOR + MC2, OUTPUT);
    pinMode(LEFT_MOTOR + EN, OUTPUT);
    
  Serial.begin(115200);
  Serial.print("ready");                   
}

int right;
int left;
int oldRight = -101;
int oldLeft = -101;
int dummy =0;
void loop(){
    if (acc.isConnected()) { 
  
      //read the received data into the byte array  
      int len = acc.read(rcvmsg, sizeof(rcvmsg), 1);    
      
      // If we've got a message...
     // if (adkCounter == 0 && len > 0) {    
       if(len > 0){  
         Serial.print("m ");
       // ...parse it     
        parseIncomingMessage(len);    
    }
    else{
      Serial.print(dummy);
      dummy = (dummy+1) % 10;
      Serial.print("x\n");
    }
    //if(weCounter == 0)
    //  checkWE();
      
    //if(irCounter == 0)
    //  checkIR();
      
    // advance the counters (modulo their intervals)
    ++adkCounter %= ADK_INTERVAL;
    ++weCounter %= WE_INTERVAL;
    ++irCounter %= IR_INTERVAL;
  
    delay(LOOP_DELAY);
  }
  else{
    // If we lose the connection, STOP!
    left = right = 0;
    steer();
  }
}
void parseIncomingMessage(int len){
        if (rcvmsg[0] == START_BYTE) { 
        right = rcvmsg[1];
        left = rcvmsg[2];
      }      
    Serial.println("here");
    Serial.println(right);
      // The move trigger on the Android is sensitive.
      // Only worry about when the values change
       if(oldRight != right || oldLeft != left){
          steer();
          oldLeft = left;
          oldRight = right;
        }
}

// Track the WheelEncoder states 
int wheelStatesOld = 0;
// 0 = both wheels blocked
// 1 = right wheel free, left wheel blocked
// 2 = left wheel free, right wheel blocked
// 3 = both wheetls free
// read values from the Wheel encoders - global to avoid extra allocations
int wheelStates;
// IMPORTANT:  THE SAMPLING FREQUENCY MUST BE AT LEAST TWICE THE SPEED AT
// WHICH THE ENCODER CHANGES STATES
void checkWE(){
  wheelStates = 2*(analogRead(LW_ENCODER) > WE_THRESH) + (analogRead(RW_ENCODER) > WE_THRESH);
  
  if(wheelStates != wheelStatesOld){
    sndmsg[0] = OUTPUT_WE;
    sndmsg[1] = wheelStates;
    // which way are the wheels turning?
    // encode 1 positive, 0 negative, right wheel LSB, left wheel LSB+1
    sndmsg[2] =  (left > 0)*2 +(right > 0);
    acc.write(sndmsg, 3);
  }
  wheelStatesOld = wheelStates;
}

// Init IR sensor values to 1024 - i.e. HIGH reading (nothing in range)
int frontIRVal = 1024, leftFrontIRVal = 1024, rightFrontIRVal = 1024, leftIRVal = 1024, rightIRVal = 1024;

void checkIR(){
  // read values from the IR sensors  
  frontIRVal = analogRead(IRS_FRONT);
  leftFrontIRVal = analogRead(IRS_LFRONT);
  rightFrontIRVal = analogRead(IRS_RFRONT);
  leftIRVal = analogRead(IRS_LEFT);
  rightIRVal = analogRead(IRS_RIGHT);
  
  sndmsg[0] = OUTPUT_IR;
  // Saturate to max 512, scale to byte and  sensor reading
  sndmsg[1] = (byte)(min(512, frontIRVal) / 2);
  // Don't forget to ID the sensor
  sndmsg[2] = IRS_FRONT;
  acc.write(sndmsg, 3);
  
  // Saturate to max 512, scale to byte and  sensor reading
  sndmsg[1] = (byte)(min(512, leftFrontIRVal) / 2);
  // Don't forget to ID the sensor
  sndmsg[2] = IRS_LFRONT;
  acc.write(sndmsg, 3);
  
  // Saturate to max 512, scale to byte and  sensor reading
  sndmsg[1] = (byte)(min(512, rightFrontIRVal) / 2);
  // Don't forget to ID the sensor
  sndmsg[2] = IRS_RFRONT;
  acc.write(sndmsg, 3);
  
  // Saturate to max 512, scale to byte and  sensor reading
  sndmsg[1] = (byte)(min(512, leftIRVal) / 2);
  // Don't forget to ID the sensor
  sndmsg[2] = IRS_LEFT;
  acc.write(sndmsg, 3);
  
  // Saturate to max 512, scale to byte and  sensor reading
  sndmsg[1] = (byte)(min(512, rightIRVal) / 2);
  // Don't forget to ID the sensor
  sndmsg[2] = IRS_RIGHT;
  acc.write(sndmsg, 3);
}

void steer(){
  Serial.print("Right Motor: ");
  Serial.print(right);  
  Serial.print(", Left Motor: ");
  Serial.println(left);
  // How much power to right moter 
 if(right == 0){
   stop(RIGHT_MOTOR);
 }
 else if(right < 128){
   forward(right, RIGHT_MOTOR);
 } 
 // right > 128 - negative int cast to byte
 else{
   reverse(256-right, RIGHT_MOTOR);
 }
 
 // How much power to left motor
  if(left == 0){
   stop(LEFT_MOTOR);
 }
 else if(left < 128){
   forward(left, LEFT_MOTOR);
 } 
 // right > 128 - negative int cast to byte
 else{
   reverse(256-left, LEFT_MOTOR);
 }
}

// Motor goes forward 
void forward(int rate, int offset){
  // We should be getting a percentage of total power to use on 
  // this motor (i.e. 0-100).  Just to be safe, clip to that range,
  // then map to the 255 available motor power levels.
  rate = map(constrain(rate, 0, 100), 0, 100, 0, 150);
  Serial.print(offset==2 ? "left" : "right");
  Serial.print(" wheels forward at ");
  Serial.println(rate);
  digitalWrite(EN+offset, LOW);
  digitalWrite(MC1+offset, HIGH);
  digitalWrite(MC2+offset, LOW);
  analogWrite(EN+offset, rate);
}

// Motor goes backward
void reverse(int rate, int offset){
  // We should be getting a percentage of total power to use on 
  // this motor (i.e. 0-100).  Just to be safe, clip to that range,
  // then map to the 255 available motor power levels.
  rate = map(constrain(rate, 0, 100), 0, 100, 0, 150);
  
  digitalWrite(EN+offset, LOW);
  digitalWrite(MC1+offset, LOW);
  digitalWrite(MC2+offset, HIGH);
  analogWrite(EN+offset, rate);
}

// Motor stop
void stop(int offset){
  digitalWrite(EN+offset, LOW);
  digitalWrite(MC1+offset, LOW);
  digitalWrite(MC2+offset, LOW);
  digitalWrite(EN+offset, HIGH);
}
