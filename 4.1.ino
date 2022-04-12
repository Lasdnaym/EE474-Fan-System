/*4.1.c
 * @file   4.1.c
 *   @author    Corbin (full name and ID# removed for privacy)
 *   @author    Dylan (full name and ID# removed for privacy)
 *   @date      3-March-2022
 *   @brief   Lab to demonstrate understanding of the FreeRTOS Library
 *   
 *  This lab aims to familiarize us with different implementations 
 *  of the freeRTOS scheduling library
 *
 *  This early portion of the lab includes a few tasks.
 *  A task that blinks an LED, one that plays a song a couple times
 *  and a Fourer Transformation task
 *  The goal of this task to to scheudle all of these tasks effectively.
 *
 */
#include <arduinoFFT.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <stdlib.h>

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void Task_RT1( void *pvParameters );

arduinoFFT FFT = arduinoFFT();

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
double vReal[samples];
double vImag[samples];

////////////////////
//Declare Q handler
static TaskHandle_t xHandlep1, xHandlep0, xHandleT2;
static TaskHandle_t xHandlep2;
static QueueHandle_t Q_One;
static QueueHandle_t Q_Two;

// the setup function runs once when you press reset or power the board
/**
  *@brief setup() This function instantiates each pin used by our program.
  * It was also used to begin the serial outputs for our FFT task.
*/
void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  } 


  xTaskCreate(
    TaskRT_1
    ,  "Blink OFF-Board"
    ,  500  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  NULL );

  xTaskCreate(
    TaskRT_2
    ,  "Play Song"
    ,  500  // Stack size
    ,  NULL
    ,  5  // Priority
    ,  &xHandleT2 );

  xTaskCreate(
    RT3p0 
    ,  "FFT Part 0"
    ,  1500  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &xHandlep0 );

  xTaskCreate(
    RT3p1 
    ,  "FFT Part 1"
    ,  1000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &xHandlep1 );

  xTaskCreate(
    RT4 
    ,  "FFT Part 2"
    ,  1000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &xHandlep2 );

    
 // Q_Notes = xQueueCreate(2, 5 * sizeof(int));

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  //  (note how the above comment is WRONG!!!)
  vTaskStartScheduler();


}

/**
  *@brief loop() This function is entirely empty because FreeRTOS
  *will handle the scheduling of the tasks.
*/
void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/**
  *@brief TaskRT_1() This task will blink an offboard LED for 100ms and be off for 200
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void TaskRT_1(void *pvParameters){
  pinMode(47, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(47, HIGH);   // turn the LED off (HIGH is the voltage level)
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(47, LOW);    // turn the LED on by making the voltage LOW
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
  }
}

const int song[5] = {3412, 3038, 3830, 7691, 5101};

/**
  *@brief speaker() This function will play the speaker at any frequency
  *@param int TOP is the value that the clock will reach before the clock lowers.
  *This controls the frequency
*/
void speaker(int TOP){
  TCCR4A |= 0b01000000;
  TCCR4A &= ~(0b10000011);

  TCCR4B |= 0b00001010;
  TCCR4B &= ~(0b00010101);
  OCR4A = TOP;

}

/**
  *@brief TaskRT_2() This task will play the song three times before stopping 
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void TaskRT_2(void *pvParameters) {
  
  pinMode(6, OUTPUT);
  int j = 0;
  for (;;) // A Task shall never return or exit.
  {
    if(j < 3){  
      speaker(song[0]);
      vTaskDelay(500/portTICK_PERIOD_MS);
      speaker(song[1]);
      vTaskDelay(500/portTICK_PERIOD_MS);
      speaker(song[2]);
      vTaskDelay(500/portTICK_PERIOD_MS);
      speaker(song[3]);
      vTaskDelay(500/portTICK_PERIOD_MS);
      speaker(song[4]);
      vTaskDelay(500/portTICK_PERIOD_MS);
      j++;
    }
    else{
      speaker(0);
      vTaskDelete(xHandleT2);
    }
  }
}

/**
  *@brief RT3p0() This task will start the queueing process for task 3. 
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void RT3p0 (void *pvParameters){
  
  double val = randN(RAND_MAX);
  Q_One = xQueueCreate(2, sizeof(double));//val
  xQueueSendToBack(Q_One, &val, portMAX_DELAY);
  
  
  for (;;) // A Task shall never return or exit.
  {
    
    vTaskResume(xHandlep1);
    vTaskDelete(xHandlep0);
    
  }
}

/**
  *@brief RT3p1() This task will send values to a queue to later be computated
  *by the task RT4. This task also prints out how long it took the FFT function
  *to compute by RT4.
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void RT3p1 (void *pvParameters){
  unsigned long wallTime = 0;
  unsigned long current = 0;

  Q_Two = xQueueCreate(2, sizeof(double));//time
  Serial.println("Starting 5 FFTs");
  for (;;) // A Task shall never return or exit.
  {
    for(int i = 0; i < 5; i++){
      xQueueReceive(Q_Two, &current, portMAX_DELAY);
      wallTime += current;
      double val = randN(RAND_MAX);
      xQueueSendToBack(Q_One, &(val), portMAX_DELAY);
    }
    Serial.print("Wall Time for FFTs: ");
    Serial.print((wallTime / 1000.0));
    Serial.println("ms");
    wallTime = 0;
  }

}

/**
  *@brief RT4() This task will compute a FFT using 128 samples. These values are
  *sent from RT3p1. The times it took to complete are then sent back to RT3p1.
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void RT4 ( void *pvParamenters){
  double* inputs;
  unsigned long current;
  for(;;){
    xQueueReceive(Q_One, &inputs, portMAX_DELAY);
    current = micros();

    for(int i = 0; i < samples; i++){
      vImag[i] = 0.0; //manually setting to 0s
      vReal[i] = *(inputs);
    }

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, samples);
    current = micros() - current;
    xQueueSendToBack(Q_Two, &current, portMAX_DELAY);
    
  }
}

/**
  *@brief TaskBlink() This task will blink an onboard LED for 100ms and be off for 200
  *@param void *pvParameters is set to the address of a variable then the variable must 
  *still exist when the created task executes 
*/
void TaskBlink(void *pvParameters)  // This is a task.
{
 // (void) pvParameters;  // allocate stack space for params


  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for one second
  }
}

/**
  *@brief randN() This function will return a random dounle fron 1 to n
  *@param int n is the highest random double that can be returned
*/
double randN(int n)   //  return a random integer between 1 and n
{
    double x;
    x = 1.0 + (double) n * rand() / RAND_MAX;
    return((double)x);
}
