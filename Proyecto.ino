// Pin names:

#define encoderPin 2
#define out1Pin 8
#define out2Pin 9
#define pwmPin 10
#define valuesPin 11
#define potPin 15

// Variables:
const short n = 5;
const short rules[n][n] = {{0,0,0,0,0}, {0,0,0,1,1}, {0,1,1,2,2}, {1,1,2,3,4}, {1,2,3,4,4}};
boolean setValues;
int pot, pwm, i, incPWM;
volatile long encoderCounter[2];
unsigned long t[2];
float measuredPos, desiredPos, errorPos, measuredVel, desiredVel, errorVel;
float cErrorPos[n], cErrorVel[n], cPWM[n], temp[n], num, den, m;

void setup() {
  // Inicializacion de variables:
  Serial.begin(250000);
  pinMode(out1Pin,OUTPUT);
  pinMode(out2Pin,OUTPUT);
  setValues = false;
  desiredPos = 0;
  errorPos = 0;
  encoderCounter[0] = 0;
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderRead, RISING); 
}

void loop () {
  if (setValues) {
    // Stops motor:
    analogWrite(pwmPin,0);
    // Set values in serial print:
    Serial.println("********************************************");
    desiredPos = keyboardFloatInput("Enter a desired position [-90,90]: ");
    if (desiredPos<-90) { desiredPos = -90; }
    else if (desiredPos>90) { desiredPos = 90; }
    Serial.println(desiredPos);
    desiredVel = keyboardFloatInput("Enter a desired velocity [90,170]: ");
    if (desiredVel<90) { desiredVel = 90; }
    else if (desiredVel>170) { desiredVel = 170; }
    Serial.println(desiredVel);
    Serial.println("********************************************");
    setValues = false;
    delay(1000);
    encoderCounter[0] = 0;
  } else {
    // Measure of position with emergency stop:
    do {
      pot = analogRead(potPin);
      measuredPos = 0.00008389*pow(pot,2) + 0.1773*pot -132.9;      
      if (measuredPos<-100 || measuredPos>100) { analogWrite(pwmPin,0); } // Emergency stop
    } while (measuredPos<-100 || measuredPos>100);
    // Measure of velocity:
    t[1] = t[0]; t[0] = micros();
    measuredVel = 10000000*float(encoderCounter[0] - encoderCounter[1])/(4*float(t[0]-t[1])); // DPS
    measuredVel = 60*measuredVel/390; // RPM
    encoderCounter[1] = encoderCounter[0];
    // Calculus of errors:
    errorPos = desiredPos - measuredPos;
    errorVel = desiredVel - measuredVel;
    // Fuzzification:
    cErrorPos[0] = triangle(-45,0,45,abs(errorPos));
    cErrorPos[1] = triangle(0,45,90,abs(errorPos));
    cErrorPos[2] = triangle(45,90,135,abs(errorPos));
    cErrorPos[3] = triangle(90,135,180,abs(errorPos));
    cErrorPos[4] = triangle(135,180,225,abs(errorPos));
    cErrorVel[0] = triangle(-142.5,-80,-17.5,errorVel);
    cErrorVel[1] = triangle(-80,-17.5,45,errorVel);
    cErrorVel[2] = triangle(-17.5,45,107.5,errorVel);
    cErrorVel[3] = triangle(45,107.5,170,errorVel);
    cErrorVel[4] = triangle(107.5,170,232.5,errorVel);
    // Inference rules:
    for (i = 0; i < n; i++) { cPWM[i] = 0; }
    for (i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        cPWM[rules[i][j]] = max(min(cErrorVel[i],cErrorPos[j]),cPWM[rules[i][j]]);
      }
    }
    // Defuzzification:
    num = 0; den = 0;
    for(i = 0; i < n; i++) { temp[i] = 0; }
    for (int k = 0; k < 255; k++) {
      temp[0] = min(triangle(-45,-30,-15,k),cPWM[0]);
      temp[1] = min(triangle(-30,-15,0,k),cPWM[1]);
      temp[2] = min(triangle(-15,0,15,k),cPWM[2]);
      temp[3] = min(triangle(0,15,30,k),cPWM[3]);
      temp[4] = min(triangle(15,30,45,k),cPWM[4]);
      m = maxVec(temp);
      num += k*m;
      den += m;
    }
    incPWM = round(num/den);
    pwm += incPWM;
    if (pwm<0) { pwm = 0; }
    else if (pwm>255) { pwm = 255; }
    if (abs(errorPos) < 15.0) { pwm = map(abs(errorPos),0,15,20,50); } // Reduces speed at the end
    if (abs(errorPos) < 1.5) { pwm = 0; } // Stops with 1.5 of error
    // Actions:
    if (errorPos<0) { digitalWrite(out1Pin,HIGH); digitalWrite(out2Pin,LOW); }
    else { digitalWrite(out1Pin,LOW); digitalWrite(out2Pin,HIGH); }
    analogWrite(pwmPin,pwm);
    // Print results:
    Serial.print("dPos = ");
    Serial.print(desiredPos,5);
    Serial.print("\tmPos = ");
    Serial.print(measuredPos,5);
    Serial.print("\tePos = ");
    Serial.println(errorPos,5);
    Serial.print("errorPos = [");
    for (i = 0; i < n; i++) { 
      Serial.print(cErrorPos[i]); 
      if (i != n) { Serial.print(", "); }
    }
    Serial.println("]");
    Serial.print("dVel = ");
    Serial.print(desiredVel,5);
    Serial.print("\tmVel = ");
    Serial.print(measuredVel,5);
    Serial.print("\teVel =: ");
    Serial.println(errorVel,5);
    Serial.print("errorVel = [");
    for (i = 0; i < n; i++) { 
      Serial.print(cErrorVel[i]); 
      if (i != n) { Serial.print(", "); }
    }
    Serial.println("]");
    Serial.print("incPWM = ");
    Serial.print(incPWM);
    Serial.print("\tPWM = ");
    Serial.println(pwm);
    Serial.print("incPWM = [");
    for (i = 0; i < n; i++) { 
      Serial.print(cPWM[i]); 
      if (i != n) { Serial.print(", "); }
    }
    Serial.println("]");
    // setValues condition:
    if (digitalRead(valuesPin)) { setValues = true; }
  }
  
}

void encoderRead() { encoderCounter[0]++; }

int keyboardFloatInput(String message) {
  // Escribe un mensaje y detiene el programa hasta conseguir un valor numerico:
  Serial.println(message);
  boolean complete = false;
  int var;
  while (!complete) {
    while (Serial.available()) {
      var = Serial.parseFloat();
      char inChar = (char)Serial.read();
      if (inChar == '\n') {
        complete = true;
      }
    }
  }
  return var;
}

float triangle(float A, float B, float C, float x) {
  float m;
  if (x > A && x < B) { m = (x-A)/(B-A); }
  else if (x >= B && x < C) { m = (C-x)/(C-B); }
  else { m = 0; }
  return m;
}

float maxVec(float* v) {
  float maximum = v[0];
  for (int j = 1; j < n; j++) {
    maximum = max(maximum,v[j]);
  }
  return maximum;
}
