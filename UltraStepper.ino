#include <hcsr04.h>
#include <Stepper.h>
#include <Servo.h>
#include <math.h>

#define TRIG_PIN 12
#define ECHO_PIN 13

//Initialise ultrasound sensor
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
volatile int mmDistance = 75;

//Instantiate stepper motor
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int revPerMinute = 10;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

//Initialise servo
int potPin = 0;
int potValue;
Servo myservo;  // create servo object to control a servo

//Set point and error variables
int desiredDistance = 75; //mm
int error;
float shaftLength = 150;  //mm length of shaft on stepper motor
float errorAngle;         //In degrees
int stepsPerRad = stepsPerRevolution/(2*M_PI);
int errorSteps;           //Small angle approximation applied
int absStep;

//Setup Timer 2
int timerCount;
int mainFreq = 1000;   //Hz
int freqDivisor = 10;
int freqLimit = mainFreq/freqDivisor;
volatile int freqCounter = 0;
volatile bool freqFlag = false;
int stepLimit = mainFreq/100;
volatile int stepCounter = 0;
volatile bool stepFlag = false;

void setup() {
  noInterrupts();
  myStepper.setSpeed(revPerMinute);
  myservo.attach(5);
  Serial.begin(9600);
  // Wait for the HCSR04 sensor to settle.
  // This usually needs about one second...
  delay(1000);


// Initialize timer 2
  TCCR2A = 0;
  TCCR2B = 0;
  // Set timerCount to the correct value for our interrupt interval
  // preload timer (65536 - 16MHz/(prescaler*freq))
  timerCount = 16000000/64; //prescaler is 64
  timerCount = timerCount/1000; //desired frequency is 1000 Hz 
  timerCount = 256 - timerCount; // 8 bit timer has 256 steps
  TCNT2 = timerCount;  // preload timer
  TCCR2B |= (1 << CS22);    // prescaler = 64 - on timer 2, pull high clock select bit two high
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
  interrupts(); 
}



ISR(TIMER2_OVF_vect)
{
  interrupts(); //re-enable interrupts
  // preload the timer each cycle
  TCNT2 = timerCount;
  freqCounter = freqCounter + 1;
  stepCounter = stepCounter + 1;
  
  if (stepCounter > stepLimit){
    stepCounter = 0;
    stepFlag = true;
  }
  if (freqCounter > freqLimit){
    freqCounter = 0;
    freqFlag = true;
  }
}




void loop() {

  //Read distance value if interrupt has occurred
   if (freqFlag == true){
     mmDistance = hcsr04.distanceInMillimeters();   // Read ultrasonic sensor and store value
     freqFlag = false;  
     potValue = analogRead(potPin);
     potValue = map(potValue, 0, 1023, 15, 150);
     myservo.write(potValue);
   }
   
   //Serial.print(mmDistance);
   //Serial.println(" mm");
   error = mmDistance - desiredDistance;
   errorAngle = error/shaftLength;
   errorSteps = errorAngle*stepsPerRad;
   absStep = abs(errorSteps);
   Serial.println(errorSteps);



   if (stepFlag == true){
     //Move the stepper
     if (absStep < 50){
       myStepper.step(errorSteps);
       //Serial.println("A");
     }
     else if(absStep > 200){
      //
     }
     else if(errorSteps < 0){
       myStepper.step(-1);
       //myStepper.step(errorSteps);
       //Serial.println("B");
     }   
     else if (errorSteps > 0){
       myStepper.step(1);
       //myStepper.step(errorSteps);
       //Serial.println("C");
     }
    stepFlag = false;
    errorSteps = 0;
   }
}
