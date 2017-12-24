#include <stdint.h>

#define pinOffset 3

void pinModeRange(const int16_t, const int16_t, const int16_t);
void digitalWriteRange(const int16_t, const int16_t, const int16_t);
void timeUpdate();
void buttonRisingTriggered(int16_t);

const int8_t buttonPin = 2;
const int8_t D0 = 3; 
const int8_t D1 = 4;
const int8_t D2 = 5;
const int8_t D3 = 6;

const int8_t buzzerPin = 7;

const int8_t dataPin = 8;
const int8_t clockPin = 12;
const int8_t latchPin = 13;

int8_t hours = 13;
int8_t minutes = 59;
int8_t seconds = 0;

int8_t diosMin;
int8_t ditsMin;
int8_t dioshour;
int8_t ditshour;
int8_t * times[4] = {&diosMin, &ditsMin, &dioshour, &ditshour};

/*                          *\
       R&W Button State
\*                          */
int8_t lastButtonState = LOW;
int8_t buttonState;
int8_t ledState = LOW;

/*                               *\
   R&W Button Triggered Interval
\*                               */
uint32_t lastDebounceTime = 0;
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

void setup() {
  pinMode(buttonPin, INPUT);
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
    }
    hourSum = space[0]*10 + space[1];
    minuteSum = space[2]*10 + space[3];
    if(hourSum < 24 && minuteSum < 60){
      hours = hourSum;
      minutes = minuteSum;
      seconds = 0;
    } 
  }
  
  if(millis() - lastInterval >= 1000){
    // Time
    lastInterval = millis();

    diosMin = minutes % 10; // Digit in ones of Minutes
    ditsMin = minutes / 10; // Digit in tens of Minutes
    dioshour = hours % 10; // Digit in ones of Hours
    ditshour = hours / 10; // Digit in tens of Hours
       
    seconds++;
    if(seconds>=60) timeUpdate();
  }

  // buttonState
  int16_t reading = digitalRead(buttonPin);
  
  buttonRisingTriggered(reading);
  
  // When ledState is LOW, D0~D3 will be enabled.
  digitalWriteRange(D0, D3, ledState);
  
  if(ledState == LOW) enableDx((count % 4) + pinOffset, *times[count]);
  count++;
  count %= 4;
  delay(5);
  
  lastButtonState = reading;
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

void pinModeRange(const int16_t pinS, const int16_t pinE, const int16_t mode){
  for(int i=pinS; i<=pinE; i++) pinMode(i, mode);
}

void digitalWriteRange(const int16_t pinS, const int16_t pinE, const int16_t state){
  for(int i=pinS; i<=pinE; i++) digitalWrite(i, state);
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

void buttonRisingTriggered(int16_t reading){
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
}

