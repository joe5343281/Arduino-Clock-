#include <stdint.h>

#define pinOffset 3

void pinModeRange(const int16_t, const int16_t, const int16_t);
void digitalWriteRange(const int16_t, const int16_t, const int16_t);
void timeUpdate();
void buttonRisingTriggered(int16_t);

const int8_t redButtonPin = 2;
const int8_t whiteButtonPin = 9;
const int8_t D0 = 3; 
const int8_t D1 = 4;
const int8_t D2 = 5;
const int8_t D3 = 6;

const int8_t buzzerPin = 7;

const int8_t dataPin = 8;
const int8_t clockPin = 12;
const int8_t latchPin = 13;

int8_t hours = 20;
int8_t minutes = 17;
int8_t seconds = 0;

int8_t diosMin;
int8_t ditsMin;
int8_t dioshour;
int8_t ditshour;
int8_t * times[4] = {&diosMin, &ditsMin, &dioshour, &ditshour};

/*                          *\
       R&W Button State
\*                          */
int8_t lastRedButtonState = LOW;
int8_t redButtonState;
int8_t ledState = LOW;

int8_t lastWhiteButtonState = LOW;
int8_t whiteButtonState;

/*                               *\
   R&W Button Triggered Interval
\*                               */
uint32_t lastRedDebounceTime = 0;
uint32_t lastWhiteDebounceTime = 0;
uint32_t debounceDelay = 25;

uint8_t count = 0;

uint32_t lastInterval = 0;

/*
                            a, b, c, d, e, f, g  common cathode
const int seven_seg[10][7] =  {
                              {0, 0, 0, 0, 0, 0, 1}, // 0
                              {1, 0, 0, 1, 1, 1, 1}, // 1
                              {0, 0, 1, 0, 0, 1, 0}, // 2
                              {0, 0, 0, 0, 1, 1, 0}, // 3
                              {1, 0, 0, 1, 1, 0, 0}, // 4
                              {0, 1, 0, 0, 1, 0, 0}, // 5
                              {0, 1, 0, 0, 0, 0, 0}, // 6
                              {0, 0, 0, 1, 1, 1, 1}, // 7
                              {0, 0, 0, 0, 0, 0, 0}, // 8
                              {0, 0, 0, 0, 1, 0, 0}, // 9
                                                  };
*/

const int seven_seg_digits[10] =  {
                                    B1111110,  // = 0
                                    B0110000,  // = 1
                                    B1101101,  // = 2
                                    B1111001,  // = 3
                                    B0110011,  // = 4
                                    B1011011,  // = 5
                                    B1011111,  // = 6
                                    B1110000,  // = 7
                                    B1111111,  // = 8
                                    B1110011   // = 9
                                  };

// Note frequency
#define Do  261 // 1
#define Re  293 // 2
#define Me  329 // 3
#define Fa  349 // 4
#define Sol 392 // 5
#define La  440 // 6
#define Si  493 // 7


const int16_t christmasSong[] = {Me, Me, Me, 0, Me, Me, Me, 0, Me, Sol, Do, Re, Me, 0,
                                 Fa, Fa, Fa, Fa, Fa, Me, Me, Me, Me, Re, Re, Me, Re, 0, Sol, 0,
                                 Me, Me, Me, 0, Me, Me, Me, 0, Me, Sol, Do, Re, Me, 0,
                                 Fa, Fa, Fa, Fa, Fa, Me, Me, Me, Sol, Sol, Fa, Re, Do
                                 
};

void setup() {
  pinMode(redButtonPin, INPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  
  pinModeRange(D0, D3, OUTPUT);
  digitalWriteRange(D0, D3, HIGH);

  Serial.begin(9600);
}

void loop() {
  int16_t space[4] = {0, 0, 0, 0};
  

  while(Serial.available()){
    int8_t digits = 0;
    int8_t hourSum = 24;
    int8_t minuteSum = 60;
    while(digits<4){
      int16_t oriData = Serial.read();
      int16_t handledData = oriData - '0';
      if(handledData >= 0 && handledData <= 9){
      space[digits] =  handledData;  
      digits++;
      }
      Serial.println(handledData);
    }
    hourSum = space[0]*10 + space[1];
    minuteSum = space[2]*10 + space[3];
    if(hourSum < 24 && minuteSum < 60){
      hours = hourSum;
      minutes = minuteSum;
      seconds = 0;
    } 
  }
  
  addOneSencond();

  // buttonState
  int16_t redReading = digitalRead(redButtonPin);
  int16_t whiteReading = digitalRead(whiteButtonPin);

  buttonRisingTriggered(redReading, redButtonPin, &redButtonState, &lastRedButtonState, &lastRedDebounceTime);
  buttonRisingTriggered(whiteReading, whiteButtonPin, &whiteButtonState, &lastWhiteButtonState, &lastWhiteDebounceTime);
  //buttonRisingTriggered(reading);
  
  // When ledState is LOW, D0~D3 will be enabled.
  digitalWriteRange(D0, D3, ledState);
  
  showTime(ledState);
  
  lastRedButtonState = redReading;
  lastWhiteButtonState = whiteReading;
}

void enableDx(const int8_t Dx, int8_t number){
  for(int i=D0; i<=D3; i++){
    if(i != Dx) digitalWrite(i, HIGH);
    else  digitalWrite(i, LOW);
  }
  // Send Dx data to 
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, seven_seg_digits[number]);
  digitalWrite(latchPin, HIGH);
}
void showTime(int8_t ledState){
  if(ledState == LOW) enableDx((count % 4) + pinOffset, *times[count]);
  count++;
  count %= 4;
  delay(5);
}

void timeUpdate(){
  if(seconds / 60){
    seconds %= 60;
    minutes++;
    if(minutes / 60){
      minutes %= 60;
      hours++;
      if(hours / 24) hours %= 24;
      if(!ledState){
        tone(buzzerPin, 429, 100);
        delay(100);
      }     
    }    
  }
}

void computeTime(){
  diosMin = minutes % 10; // Digit in ones of Minutes
  ditsMin = minutes / 10; // Digit in tens of Minutes
  dioshour = hours % 10; // Digit in ones of Hours
  ditshour = hours / 10; // Digit in tens of Hours
}

void addOneSencond(){
  if(millis() - lastInterval >= 1000){
    // Time
    lastInterval = millis();

    seconds++;
    if(seconds>=60) timeUpdate();
    computeTime();   
  }
}

void buttonRisingTriggered(int16_t reading, const int8_t buttonPin, int8_t *buttonState, int8_t *lastButtonState,  int32_t *lastDebounceTime){
  if (reading != *lastButtonState) {
    *lastDebounceTime = millis();
  }

  if ((millis() - *lastDebounceTime) > debounceDelay) {
    if (reading != *buttonState) {
      *buttonState = reading;
      if(buttonPin == redButtonPin){
        if(*buttonState == HIGH) ledState = !ledState;
      }
      else{
        if(*buttonState == HIGH) {
          playMusic();
          
        }
      }
    }
  }
}

void pinModeRange(const int16_t pinS, const int16_t pinE, const int16_t mode){
  for(int i=pinS; i<=pinE; i++) pinMode(i, mode);
}

void digitalWriteRange(const int16_t pinS, const int16_t pinE, const int16_t state){
  for(int i=pinS; i<=pinE; i++) digitalWrite(i, state);
}

void playMusic(){
  int16_t len = (sizeof christmasSong) / sizeof(int16_t);
  for(int16_t i=0; i<len; i++){
    tone(buzzerPin, christmasSong[i], 180);
    delay(180);
    addOneSencond();
    showTime(ledState);
    Serial.println(seconds);
  }
}

