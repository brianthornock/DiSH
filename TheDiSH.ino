#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins
const unsigned int tapSwitch = 1;
const unsigned int ledOut = 2;
const unsigned int vOut = 4;
const unsigned int encA = 3;
const unsigned int encB = 0;
const unsigned int encSW = 5;

// Initiate encoder stuff
volatile int lastEnc = 0;

// Initiate the tap tempo button stuff
int buttonState;
int lastButtonState = LOW; // Previous state of tap tempo switch
int tapStatus = LOW; // Current status of the tap tempo switch

unsigned long lastDebounceTime = 0; // initiate the debounce time counter
unsigned long debounceDelay = 100; // How long we enforce a debounce

unsigned int prevTaps = 0; // How many taps we have in our history
unsigned int tapTimeout; // numSeconds*1000
unsigned int maxTaps = 10; // Max number of taps to keep in history
unsigned int prevTimes [10]; // Array for storing the tap periods
unsigned int tapTime; // Averaged value of times in tap history
unsigned int prevTapTime; // Previous averaged tap tempo time
unsigned int useTap; // This is used to determine if we have enough taps to calculate the tap period
unsigned int prevTapDelay; // This is used for debouncing the tap tempo switch
unsigned int numSteps = 128; //number of divisions allowable between the maximum and minimum hold period

//Set up bounds for how fast/slow we allow things to go before divisions
unsigned long minFreq = 0.5; // Minimum frequency in Hz
unsigned int maxFreq = 20; // Maximum frequency in Hz
unsigned int minTime; // shortest allowable period between changes in voltage level
unsigned int maxTime; // longest allowable period between changes in voltage level
unsigned int timeStep; // The amount of time between divisions over the range of [minTime:maxTime]
unsigned int usePattern; //Whether or not we are using the pattern, determined by encoder switch
unsigned int encSWCount = 0; // Counter for how many times the switch has been pressed. Used for determining multiplier/pattern
unsigned int prevSWState = LOW;

unsigned int defaultSampleTime; //duration in ms of a 120 bpm default tempo
unsigned int sampleTime; //set this to the default
unsigned int prevSampleTime; //set this to the default
unsigned int currentMillis; // Used for debouncing tap tempo switch
unsigned int prevMillis; // Used for keeping track of LED blinking
unsigned int prevVoltMillis; // Used for keeping track of voltage
unsigned int prevPatternMillis = 0; // Used for keeping track of pattern step time
unsigned int patternSampleTime = 0;
unsigned int maxLEDTime = 300; //Maximum LED blink on duration in ms
unsigned int currLEDState = LOW; // LED starts off off
unsigned int currLEDOffInterval; // How long the LED has been off
unsigned int currLEDOnInterval; // How long the LED has been on
unsigned int blinkDuration; // How long should the LED be on
unsigned int currVoltInterval; // How long have we held the current voltage level
unsigned int voltDuration; // How long do we need to hold the current voltage level

unsigned int multiplier = 1; // How much to divide the tapped tempo
unsigned int prevMultiplier = 1; // Last multiplier for checking to see what we need to do
unsigned int patternMultiplier = 1; // Keeps track of the current multiplier for the pattern function
unsigned int prevTempo; // Last tempo that we indicated
unsigned int patternCount = 0; // Indicates were in our tempo pattern we are

const unsigned int minPWM = 10; //Minimum PWM duty cycle
const unsigned int minPWMDelta = 10; //Minimum separation between voltage levels
unsigned int prevDutyCycle;

unsigned int updateMult = 0;
unsigned int updateLEDInterval = 1;

void setup() {
  //Define what each pin is
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(encSW, INPUT_PULLUP);
  pinMode(tapSwitch, INPUT_PULLUP);
  pinMode(vOut, OUTPUT);
  pinMode(ledOut, OUTPUT);

  //Set up the initial state of the pins
  digitalWrite(encA, HIGH);
  digitalWrite(encB, HIGH);
  digitalWrite(encSW, LOW);
  digitalWrite(tapSwitch, LOW);
  digitalWrite(ledOut, LOW);
  digitalWrite(vOut, LOW);

  defaultSampleTime = round(60 / 120 * 1000); //duration in ms of a 120 bpm default tempo
  sampleTime = round(60 / 120 * 1000); //set this to the default
  prevSampleTime = round(60 / 120 * 1000); //set this to the default
  tapTime = 500;
  prevTapTime = 500;
  prevTapDelay = 0;

  prevMillis = millis();
  prevVoltMillis = millis();

  //Initialize the voltage out
  int newDutyCycle = random(minPWM,255);
  prevDutyCycle = newDutyCycle;

  //Update the output PWM duty cycle
  analogWrite(vOut, newDutyCycle);

  minTime = 50; // 1/maxFreq*1000 shortest allowable period between changes in voltage level
  maxTime = 5000; // round(1 / minFreq * 1000); // longest allowable period between changes in voltage level
  timeStep = round(4950 / 1024); // The amount of time between divisions over the range of [minTime:maxTime]
  tapTimeout = 2500; // How long to keep waiting for taps

  TCCR1 = (TCCR1 & 0b11110000) | 0b0001;
  //Set up an ISR for the encoder switch pin so that it reacts instantly
  //and to reduce loop execution time with determining multiplier
  GIMSK = 0b00100000; //enable pin change interrupts
  PCMSK = 0b00001001; //enable PB3 and PB4 as pin change interruptible
  sei(); //start interrupt service
}



void loop() {
  //Update the multiplier by polling the encoder switch
  updateMultiplier();

  //Check to see if tap tempo is used
  checkTapTempo();

  //Update the sample time based on above
  updateSampleTime();

  currentMillis = millis();

  //Update the random voltage generation
  voltageOut();

  //Update the LED blink rate
  //Blink LED for 1/2 period with a max illumination of maxLEDTime ms per period
  updateLED();
}


//Interrupt handling
ISR (PCINT0_vect) {
  int  readA1 = digitalRead(encA);
  int  readB1 = digitalRead(encB);

  delayMicroseconds(2000);
  int  readB2 = digitalRead(encB);
  int  readA2 = digitalRead(encA);

  if (readA1 == readA2 && readB1 == readB2) {

    int MSB = readA1;
    int LSB = readB1;

    int encoded = (MSB << 1) | LSB; // shift MSB by one bit and append LSB
    int sum = (lastEnc << 2) | encoded; // shift last ENC value by two bits and append the latest two

    //Clockwise turn
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
      tapTime = (tapTime / multiplier) - timeStep;

      if (tapTime < minTime) {
        tapTime = minTime;
      }
    }
    //Counterclockwise turn
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
      tapTime = (tapTime / multiplier) + timeStep;

      if (tapTime > maxTime) {
        tapTime = maxTime;
      }
    }
    lastEnc = encoded;
  }

}



void updateMultiplier() {
  // Poll the switch, but let the encoders use the interrupts
  int currSW = digitalRead(encSW);
  int isDebounced = 0;

  if (currSW == HIGH) {
    delay(100);
    if (digitalRead(encSW) == currSW || prevSWState == LOW) {
      encSWCount++;
      updateMult = 1;
      isDebounced = 1;
      prevSWState = HIGH;
    }
    else {
      prevSWState = LOW;
    }
  }

  //Check to see what the multiplier is and if we are using the pattern
  if (updateMult && isDebounced) {
    switch (encSWCount) {
      case 0:
        multiplier = 1;
        break;
      case 1:
        multiplier = 2;
        break;
      case 2:
        multiplier = 4;
        break;
      case 3:
        multiplier = 1;
        usePattern = 1;
        break;
      case 4:
        multiplier = 1;
        usePattern = 0;
        encSWCount = 0;
        break;
    }
    isDebounced = 0;
  }
}


void updateSampleTime() {
  int tempSampleTime = tapTime;

  if (usePattern != 1) {
    //If we are more than 5 ms off and not using pattern, update the sampleTime
    if (abs(tapTime - prevTapTime) >= 5) {
      //blinkDebugLED(1, 4);
      tempSampleTime = tapTime;
      prevTapTime = tapTime;
      patternMultiplier = 1;
      encSWCount = 0; // If we change the tempo, reset encoder count and go to a multiplier of 1
      updateLEDInterval = 1;
    }
    // If the multiplier has changed, update
    else if (updateMult) {
      tempSampleTime = round(tapTime / multiplier); //Round to nearest ms
      updateMult = 0;
      prevMillis = millis();
      updateLEDInterval = 1;
    }
  }
  // If we are using the pattern, handle everything for that here
  else {
    if (millis() - prevPatternMillis >= patternSampleTime) {
      updateLEDInterval = 1;
      //create the pattern for the switch as one quarter and 2 eighth notes
      switch (patternCount) {
        case 0:
          patternMultiplier = 1;
          break;
        case 1:
          patternMultiplier = 2;
          break;
        case 2:
          patternMultiplier = 2;
          break;
        case 3:
          patternCount = 0;
          patternMultiplier = 1;
          break;
      }
      patternCount++;
      patternSampleTime = round(sampleTime / patternMultiplier);
      prevPatternMillis = millis();
    }

  }

  sampleTime = tempSampleTime;// / patternMultiplier;
}



//Code for debouncing tap tempo switch
void switchDebounce() {
  int reading = digitalRead(tapSwitch);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {

      buttonState = reading;

      if (buttonState == HIGH) {
        tapStatus = HIGH;
        //blinkDebugLED(1, 1);
      }
    }
  }

  lastButtonState = reading;
}


void checkTapTempo() {

  //Check to see if the tap tempo switch has been pressed
  switchDebounce();

  if (tapStatus == HIGH) {

    tapStatus = LOW;
    //Check to see if we already have a tap tempo history. If so, add this to
    //the history. If not, start a new count.
    if (prevTaps > 0) {
      int currTime = millis();
      int currDelay = currTime - prevTapDelay;
      // Check to make sure we didn't time out
      if (currDelay < tapTimeout) {
        //Set the flag for using tap tempo
        useTap = 1;

        // Create the temp array for storing times in
        unsigned int newPrevTimes [maxTaps];

        if (prevTaps < maxTaps) {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k];
          }

          //Then add in the new value at the end
          newPrevTimes[prevTaps - 1] = currDelay;
          prevTaps++;
          //blinkDebugLED(1, 2);


        } // End if prevTaps < maxTaps
        // If we have filled the tap buffer
        /*
          else {

          //Fill up the new array with all the old values first
          for (int k = 0; k < prevTaps - 1; k++) {
            newPrevTimes[k] = prevTimes[k + 1];
          }

          //Then add in the new value at the end
          newPrevTimes[maxTaps-1] = currDelay;
          prevTaps = maxTaps;
          }
        */
        for (int nTime = 0; nTime < maxTaps; nTime++) {
          prevTimes[nTime] = newPrevTimes[nTime];
        }

      } // End if currDelay < tapTimeout
      else {
        //If we timeout, reset the counter and zero out the tempo array
        prevTaps = 1;

        for (int i = 0; i < maxTaps; i++) {
          prevTimes[i] = 0;
        }

        useTap = 0;
      } // End if tap has timed out
    } // End if prevTaps > 0
    // If we do not have any previous taps (first tap after timeout)
    else {
      prevTaps = 1;

      for (int i = 0; i < maxTaps; i++) {
        prevTimes[i] = 0;
      }

      useTap = 0;
    }

    if (useTap == 1 && prevTaps > 2) {
      //Calculate the average polling time, including the multiplier and the random switch
      int sum, loop, numVals;
      float avg;

      sum = avg = 0;
      numVals = 0;

      for (loop = 0; loop < prevTaps - 1; loop++) {
        if (prevTimes[loop] != 0) {
          sum += prevTimes[loop];
          numVals++;

          //digitalWrite(encSW, HIGH);
          //delayMicroseconds(prevTimes[loop]);
          //digitalWrite(encSW, LOW);
        }
      }
      avg = (float)sum / numVals;
      tapTime = round(avg);
      multiplier = 1;
      encSWCount = 0;
      //blinkDebugLED(1,3);

    }
    else {
      //If we don't have the information to produce a tap tempo, stick with what we have
    }
    prevTapDelay = millis();
    //blinkDebugLED(1,3);
  }

}




//Code to produce the voltage
void voltageOut() {

  //If we have reached sampleTime or we have changed how long to hold, get a new voltage. Otherwise, don't do anything
  if (currentMillis - prevVoltMillis >= sampleTime || updateLEDInterval) {
    // Create the new duty cycle. PWM operates on 0-255 range for duty cycle
    int newDutyCycle = random(minPWM,255);

    while (abs(newDutyCycle - prevDutyCycle)<minPWMDelta){
      newDutyCycle = random(minPWM,255);
    }

    //Update the output PWM duty cycle
    analogWrite(vOut, newDutyCycle);
    prevDutyCycle = newDutyCycle;
    prevVoltMillis += sampleTime;

  }
}


//Code for LED flashing update
void updateLED() {

  if (usePattern != 1) {
    if (updateLEDInterval) {
      updateLEDInterval = 0;
      //blinkDebugLED(1, 5);
      if (sampleTime / 2 >= maxLEDTime) {
        currLEDOnInterval = maxLEDTime;
      }
      else {
        currLEDOnInterval = round(sampleTime / 2);
      }
      currLEDOffInterval = round(sampleTime - currLEDOnInterval);
    }
  }
  // If we are using pattern, adjust the LED blinking to match the pattern
  else {
    if (updateLEDInterval) {
      updateLEDInterval = 0;
      //blinkDebugLED(2, 5);
      if (patternSampleTime / 2 >= maxLEDTime) {
        currLEDOnInterval = maxLEDTime;
      }
      else {
        currLEDOnInterval = round(patternSampleTime / 2);
      }
      currLEDOffInterval = round(patternSampleTime - currLEDOnInterval);
    }
  }

  //Check to see if we have completed the LED on or off interval and change if we have
  if (currLEDState == LOW) {
    if (currentMillis - prevMillis >= currLEDOffInterval) {
      currLEDState = HIGH;
      prevMillis += currLEDOffInterval;
      digitalWrite(ledOut, HIGH);
    }
  }

  if (currLEDState == HIGH) {
    if (currentMillis - prevMillis >= currLEDOnInterval) {
      currLEDState = LOW;
      prevMillis += currLEDOnInterval;
      digitalWrite(ledOut, LOW);
    }
  }

}



void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledOut, HIGH);
    delay(duration);
    digitalWrite(ledOut, LOW);
    delay(duration);
  }
}

void blinkDebugLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(encSW, HIGH);
    delay(duration);
    digitalWrite(encSW, LOW);
    delay(duration);
  }
}
