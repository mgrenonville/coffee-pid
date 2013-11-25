#include <max6675.h>
#include <PID_v1.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#define PRESSURE  A3

/*****************************************************************************
 *	SSR declaration
 *****************************************************************************/

#define SSR_MAX 10
#define SSR_PIN 6
int ssrPeriodCounter = 0;
double ssrRatio = 0;

/*****************************************************************************
 *	Thermocouple declaration
 *****************************************************************************/

#define thermoDO  A0
#define thermoCS  A1
#define thermoCLK  A2

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

/*****************************************************************************
 *	PID Declaration
 ****************************************************************************/
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=1, aggKi=0, aggKd=0;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &ssrRatio, &Setpoint, consKp, consKi, consKd, DIRECT);


double* currentVar = &aggKp;


/*****************************************************************************
 *	Setup
 ****************************************************************************/
void setup(){
  Serial.begin(9600); 
  Serial.println("CoffeePID Booting...");
  pinMode(SSR_PIN, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  /*
     * A conception error (using PD6 instead of PD5) forces to use Interrupt
   * mode on Timer1, instead of the PWM module.
   * CTC (Clear Timer on Compare) mode to compare timer with OCF1A to create an 
   * interruption when Timer1 = OCR1A and reset Timer0
   *
   * see: https://sites.google.com/site/qeewiki/books/avr-guide/timer-on-the-atmega8
   */
  TCCR1B |= (1 << WGM12);
  TIMSK1 |= (1 << OCIE1A);

  /*
     * Compute the number of cycles in 200ms (10 periods of 50Hz)
   * 	F_cpu / 200ms = 16MHz / 5Hz = 3 200 000 cycles
   *
   * With 8 prescaler : 
   *  ICR1 = 3 200 000 / 8  = 400 000 > 65535 (16bit counter)
   * With 64 prescaler
   *  ICR1 = 3 200 000 / 64 = 50 000 < 65535 (16bit counter)
   *
   * Ok, so use a 64 (CS1 = 011) prescaler with OCR1A set to 50 000.
   */

  TCCR1B |= (1 << CS11) | (1 << CS10);
  OCR1A = 50000;

  Serial.println("Timer configured, enabling interruptions...");
  /*
     * Enabling interrupts
   */
  sei();
  delay(4000);
  Serial.println("Waiting thermocouple to stabilize");
  Serial.print("Initial temperature value :");
  Serial.println(thermocouple.readCelsius());
  delay(4000);

  Serial.print("temperature value :");
  Serial.println(thermocouple.readCelsius());


  Serial.println("interruptions enabled.");
  Serial.print("Initial temperature value :");
  Serial.println(thermocouple.readCelsius());
  Setpoint = 100;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, SSR_MAX);
  myPID.SetSampleTime(2000);
  myPID.SetTunings(aggKp, aggKi, aggKd);
}



/*****************************************************************************
 *	Main Loop : 
 *  - Reads the thermocouple value
 *  - Compute PID output
 *****************************************************************************/
void loop(){

  Input = readAverageTemperature(4, 2000);
  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap < 10){  //we're close to setpoint, use conservative tuning parameters
    // myPID.SetTunings(consKp, consKi, consKd);
  } 
  else {
    //we're far from setpoint, use aggressive tuning parameters

  }

  myPID.SetTunings(aggKp, aggKi, aggKd);
  myPID.Compute();



  if(Serial.available() > 0){
    int inByte = Serial.read();
    if(inByte == 'p'){
      currentVar = &aggKp;

    } 
    else if(inByte == 'i'){
      currentVar = &aggKi;
    }
    else     if(inByte == 'd'){
      currentVar = &aggKd;
    } 
    else if(inByte == '-') {
      (*currentVar)--;
    }
    else if(inByte == '+') {
      (*currentVar)++;
    }
    else if(inByte == '/') {
      (*currentVar) =       (*currentVar) / 2 ;
    }
    else if(inByte == '*') {
      (*currentVar) =       (*currentVar) *2 ;
    }
    else if(inByte == 'm') {
      myPID.SetMode(MANUAL);
    }
    else if(inByte == 'a') {
      myPID.SetMode(AUTOMATIC);
    }

    Serial.println(*currentVar);
    Serial.print("P="); Serial.print(aggKp);    Serial.print(",I="); Serial.print(aggKi);    Serial.print(",D="); Serial.println(aggKd);
  }
}

double readAverageTemperature(int measure, int globalDelay){
  double temp = 0;
  for(int i = 0; i < measure ; i++) {

    temp += thermocouple.readCelsius();
    delay(globalDelay/measure);
  }

  double calculatedAverageTemp = temp / measure;
  Serial.print(millis());
  Serial.print("  ,  ");
  Serial.print(calculatedAverageTemp);

  Serial.print("  ,  ");
  Serial.println(ssrRatio);
  return calculatedAverageTemp;

}

/*****************************************************************************
 * The timer interruption function. Will be called every 200ms. 
 * 
 * It will compare the ssrPeriodCounter to ssrRatio and will output on SSR 
 * pin : 
 * - LOW if upper 
 * - HIGH if lower
 * 
 * Since we are driving a SSR with Zero-Crossing detection, at 5Hz, the SSR
 * would be able to dim the signal by portion of 10% (5Hz => 200ms = 10 * 20ms
 * => 10 periods of 50Hz).  We need to test at every interrupt since the value
 * of ssrRatio can change.
 * 
 * see: http://www.unitemp.com/files/images/ssrs_zerocrossing_dia.jpg
 ******************************************************************************/

ISR (TIMER1_COMPA_vect){
  ssrPeriodCounter++;
  if(ssrPeriodCounter > SSR_MAX - 1){
    ssrPeriodCounter = 0;
  }

  /* 
   */
  if(ssrPeriodCounter < ssrRatio) {
    digitalWrite(SSR_PIN, 1);
  } 
  else {
    digitalWrite(SSR_PIN, 0);
  }
}




