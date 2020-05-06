#include <PID_v1.h>

//IAV Constants
#define EN_PIN 4
#define DIR_PIN 3
#define STEP_PIN 2
#define STEPLOW 200
#define STEPHIGH 50
#define STROKEMAX 240
#define STROKEMIN 0
#define STARTSTROKE 240
#define COMMANDCYCLE 4
#define STATETIME 100

//Respiration Cycle Constants
#define BLOWERSPD_PIN 6
#define DPSENSOR_PIN A7
#define SOLENOIDCTLPIN 7
#define INH_FLOWSENSOR_PIN A6
#define EXH_FLOWSENSOR_PIN A7

//state machine variables
#define INSPIRE_TIME 500
#define PIP 400//606 // = 20cmH2O (10bit scaling)
#define EXPIRE_TIME 750
#define PEEP 150//305 // = 5cmH2O (10bit scaling)
//not implemented yet
#define AC 0
#define RR 0
#define IE 0


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
int accumulator = 0;
int setpointIAV = 0;
int sensorValue = 0;        
int outputValue = 0;        
int flowValueINH = 0;
int flowValueEXH = 0;
unsigned int cyclecounter = 0;
unsigned int state = 0;
unsigned int now = 0;


//Specify the links and initial tuning parameters
double Kp = 0.7*0.45, Ki = 0.7*0.54/0.16, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  
  // set up pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize PID
  Input = analogRead(DPSENSOR_PIN);
  Setpoint = PEEP;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(SOLENOIDCTLPIN, OUTPUT); 

  //home the actuator
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  for(int k=0; k<STARTSTROKE; k++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEPHIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEPLOW);
  }
  accumulator = 0; //set home
}

void loop() {
  
  switch(state) {

    case 0:
      cyclecounter++;
      //set command
      Setpoint = PIP;
      setpointIAV = 10;
      digitalWrite(SOLENOIDCTLPIN,1);
      //update state
      if (cyclecounter > INSPIRE_TIME) {
        cyclecounter = 0;
        state = 1;
      }
      break;
            
    case 1:
      cyclecounter++;
      //set command
      Setpoint = PEEP;
      setpointIAV = 250;
      digitalWrite(SOLENOIDCTLPIN,1);
      //update state
      if (cyclecounter > EXPIRE_TIME) {
        cyclecounter = 0;
        state = 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  //action block
  //Read sensors
  sensorValue = analogRead(DPSENSOR_PIN); //read sensor
  flowValueINH = analogRead(INH_FLOWSENSOR_PIN);
  flowValueEXH = analogRead(EXH_FLOWSENSOR_PIN);
  //Update PID Loop
  Input = sensorValue; //map to output scale
  myPID.Compute(); // compute PID command
  //setpointIAV = 2*(int)Output; //update valve command
 

  //Move valve
  //limit checks
  if(setpointIAV > STROKEMAX) { setpointIAV = STROKEMAX; }
  if(setpointIAV < STROKEMIN) { setpointIAV = STROKEMIN; }
  //stepper seeking
  digitalWrite(EN_PIN, LOW);
  for(int k=0; k<COMMANDCYCLE; k++){
    if(setpointIAV > accumulator) { 
      digitalWrite(DIR_PIN, LOW); 
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEPHIGH);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEPLOW);
      accumulator++;
    }
    if(setpointIAV < accumulator) { 
      digitalWrite(DIR_PIN, HIGH); 
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEPHIGH);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEPLOW);
      accumulator--;
    }
    else {delayMicroseconds(STEPHIGH+STEPLOW);}
  }

  //serial data out
  now = (unsigned int)millis();
  Serial.print("C"); //output to monitor
  Serial.write(now>>8);
  Serial.write(now&0xff);
  Serial.write(int(Setpoint)>>8); //output to monitor
  Serial.write(int(Setpoint)&0xff); //output to monitor
  Serial.write(int(setpointIAV)>>8); //output to monitor
  Serial.write(int(setpointIAV)&0xff); //output to monitor
  Serial.write(int(sensorValue)>>8); //output to monitor
  Serial.write(int(sensorValue)&0xff); //output to monitor

}
