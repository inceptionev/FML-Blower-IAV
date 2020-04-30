#define EN_PIN 4
#define DIR_PIN 3
#define STEP_PIN 2
#define STEPLOW 450
#define STEPHIGH 50
#define STROKEMAX 500
#define STROKEMIN 0
#define STARTSTROKE 1250
#define COMMANDCYCLE 4
#define STATETIME 250

unsigned int state = 0;
int accumulator = 0;
int setpointIAV = 0;
int cyclecounter = 0;

void setup() {
  // set up pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);

  //home the actuator
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
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
      setpointIAV = 0;
      //update state
      if(cyclecounter > STATETIME){
        cyclecounter=0;
        state = 1;
      }
      break;
      
    case 1:
      cyclecounter++;
      setpointIAV = 125;
      //update state
      if(cyclecounter > STATETIME){
        cyclecounter=0;
        state = 2;
      }
      break;

    case 2:
      cyclecounter++;
      setpointIAV = 250;
      //update state
      if(cyclecounter > STATETIME){
        cyclecounter=0;
        state = 3;
      }
      break;

     case 3:
      cyclecounter++;
      setpointIAV = 375;
      //update state
      if(cyclecounter > STATETIME){
        cyclecounter=0;
        state = 4;
      }
      break;

     case 4:
      cyclecounter++;
      setpointIAV = 500;
      //update state
      if(cyclecounter > STATETIME){
        cyclecounter=0;
        state = 0;
      }
      break;

    default:
      state = 0;
      break;
  }

  //action block

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

}
